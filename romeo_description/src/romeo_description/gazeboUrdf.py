#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This file extends urdf parsing lib (urdf.py) to handle any kind of gazebo tags

import string
from xml.dom.minidom import Document
from xml.dom import minidom
import sys
from numpy import array,pi
import re, copy

ZERO_THRESHOLD=0.000000001

def reindent(s, numSpaces):
    """Reindent a string for tree structure pretty printing."""
    s = string.split(s, '\n')
    s = [(numSpaces * ' ') + line for line in s]
    s = string.join(s, '\n')
    return s

def add(doc, base, element):
    """Add an XML element for URDF export"""
    if element is not None:
        base.appendChild( element.to_xml(doc) )

def add_openrave(doc, base, element):
    """Add an XML element for OpenRAVE XML export"""
    if element is not None:
        #TODO: copy this iterable test elsewhere
        newelements=element.to_openrave_xml(doc)
        if hasattr(newelements, '__iter__'):
            for e in newelements:
                base.appendChild(e)
        else:
            base.appendChild(newelements)

def pfloat(x):
    """Print float value as string"""
    return "{0}".format(x).rstrip('.')

def to_string(data=None):
    """Convert data fromvarious types to urdf string format"""
    if data is None:
        return None
    if hasattr(data, '__iter__'):
        outlist=[]
        for a in data:
            try:
                if abs(a)>ZERO_THRESHOLD:
                    outlist.append(pfloat(a))
                else:
                    outlist.append("0")
            except TypeError:
                outlist.append(pfloat(a))
        return ' '.join(outlist)

    elif type(data) == type(0.0):
        return pfloat(data if abs(data)>ZERO_THRESHOLD else 0)
    elif type(data) != type(''):
        return str(data)
    return data

def set_attribute(node, name, value):
    """Set an attribute on an XML node, converting data to string format"""
    node.setAttribute(name, to_string(value))

def set_content(doc,node,data):
    """Create a text node and add it to the current element"""
    if data is None:
        return
    node.appendChild(doc.createTextNode(to_string(data)))

def short(doc, name, key, value):
    element = doc.createElement(name)
    set_attribute(element, key, value)
    return element

def create_element(doc, name, contents=None, key=None, value=None):
    element = doc.createElement(name)
    if contents is not None:
        set_content(doc,element,contents)
    if key is not None:
        set_attribute(element, key, value)

    return element

def create_child(doc,name,contents=None,key=None,value=None):
    doc.appendChild(create_element(doc, name, contents=None, key=None, value=None))

def children(node):
    children = []
    for child in node.childNodes:
        if child.nodeType is node.TEXT_NODE \
                or child.nodeType is node.COMMENT_NODE:
            continue
        else:
            children.append(child)
    return children



##################################
############# Gazebo #############
##################################

class Gazebo(object):
    def __init__(self,reference=None,material=None,gravity=None,dampingFactor=None,maxVel=None,minDepth=None,mu1=None,mu2=None,fdir1=None,kp=None,kd=None,selfCollide=None,maxContacts=None,laserRetro=None,plugin=[],sensor=[]):
        self.reference = reference
        self.material = material
        self.gravity = gravity
        self.dampingFactor = dampingFactor 
        self.maxVel = maxVel
        self.minDepth = minDepth
        self.mu1 = mu1
        self.mu2 = mu2
        self.fdir1 = fdir1
        self.kp = kp
        self.kd = kd
        self.selfCollide = selfCollide
        self.maxContacts = maxContacts
        self.laserRetro = laserRetro
        self.plugins = plugin
        self.sensors = sensor

    @staticmethod
    def parse(node, verbose=True):
        print 'new gazebo tag found'
        gaze = Gazebo()
        gaze.sensors=[]
        gaze.plugins=[]
        if node.hasAttribute('reference'):
            gaze.reference = node.getAttribute('reference')
            for child in children(node):
                if child.localName == 'material':
                    gaze.material = str(child.childNodes[0].nodeValue)
                elif child.localName == 'turnGravityOff':
                    gaze.gravity = str(child.childNodes[0].nodeValue)
                elif child.localName == 'dampingFactor':
                    gaze.dampingFactor = str(child.childNodes[0].nodeValue)
                elif child.localName == 'maxVel':
                    gaze.maxVel = str(child.childNodes[0].nodeValue)
                elif child.localName == 'minDepth':
                    gaze.minDepth = str(child.childNodes[0].nodeValue)
                elif child.localName == 'mu1':
                    gaze.mu1 = str(child.childNodes[0].nodeValue)
                elif child.localName == 'mu2':
                    gaze.mu2 = str(child.childNodes[0].nodeValue)
                elif child.localName == 'fdir1':
                    gaze.fdir1 = str(child.childNodes[0].nodeValue)
                elif child.localName == 'kp':
                    gaze.kp = str(child.childNodes[0].nodeValue)
                elif child.localName == 'kd':
                    gaze.kd = str(child.childNodes[0].nodeValue)
                elif child.localName == 'selfCollide':
                    gaze.selfCollide = str(child.childNodes[0].nodeValue)
                elif child.localName == 'maxContacts':
                    gaze.maxContacts = str(child.childNodes[0].nodeValue)
                elif child.localName == 'laserRetro':
                    gaze.laserRetro = str(child.childNodes[0].nodeValue)
                elif child.localName == 'sensor':
                    gaze.sensors.append(Sensor.parse(child,verbose))       
        else:
            for child in children(node):
                if child.localName == 'plugin':
                    gaze.plugins.append(GenericPlugin.parse(child,verbose))
        return gaze
    
    def to_xml(self, doc):
        xml = doc.createElement('gazebo')
        if self.reference !=None:
            set_attribute(xml, 'reference', self.reference)
            if(self.material != None):
                xml.appendChild(create_element(doc, 'material', self.material))
            if(self.gravity != None):
                xml.appendChild(create_element(doc, 'turnGravityOff', self.gravity))
            if(self.dampingFactor != None):
                xml.appendChild(create_element(doc, 'dampingFactor', self.dampingFactor))
            if(self.maxVel != None):
                xml.appendChild(create_element(doc, 'maxVel', self.maxVel))
            if(self.minDepth != None):
                xml.appendChild(create_element(doc, 'minDepth', self.minDepth))
            if(self.mu1 != None):
                xml.appendChild(create_element(doc, 'mu1', self.mu1))
            if(self.mu2 != None):
                xml.appendChild(create_element(doc, 'mu2', self.mu2))
            if(self.fdir1 != None):
                xml.appendChild(create_element(doc, 'fdir1', self.fdir1))
            if(self.kp != None):
                xml.appendChild(create_element(doc, 'kp', self.kp))
            if(self.kd != None):
                xml.appendChild(create_element(doc, 'kd', self.kd))
            if(self.selfCollide != None):
                xml.appendChild(create_element(doc, 'selfCollide', self.selfCollide))
            if(self.maxContacts != None):
                xml.appendChild(create_element(doc, 'maxContacts', self.maxContacts))
            if(self.laserRetro != None):
                xml.appendChild(create_element(doc, 'laserRetro', self.laserRetro))
            if len(self.sensors) !=0:
                for sensor in self.sensors:
                    #print sensor
                    add(doc,xml, sensor)
        else:
            if len(self.plugins)!=0:
                for plugin in self.plugins:
                    add(doc,xml, plugin)

        return xml

    def __str__(self):
        s = ""
        if self.reference !=None:
            s += "Reference: {0}\n".format(self.reference)
            s += "Material:\n"
            s += reindent(str(self.material),1) + "\n"
            s += "Gravity:\n"
            s += reindent(str(self.gravity),1) + "\n"
            s += "DampingFactor:\n"
            s += reindent(str(self.dampingFactor),1) + "\n"
            s += "MaxVel:\n"
            s += reindent(str(self.maxVel),1) + "\n"
            s += "MinDepth:\n"
            s += reindent(str(self.minDepth),1) + "\n"
            s += "Mu1:\n"
            s += reindent(str(self.mu1),1) + "\n"
            s += "Mu2:\n"
            s += reindent(str(self.mu2),1) + "\n"
            s += "Fdir1:\n"
            s += reindent(str(self.fdir1),1) + "\n"
            s += "Kp:\n"
            s += reindent(str(self.kp),1) + "\n"
            s += "Kd:\n"
            s += reindent(str(self.kd),1) + "\n"
            s += "SelfCollide:\n"
            s += reindent(str(self.selfCollide),1) + "\n"
            s += "MaxContacts:\n"
            s += reindent(str(self.maxContacts),1) + "\n"
            s += "LaserRetro:\n"
            s += reindent(str(self.laserRetro),1) + "\n"
            for sensor in self.sensors:
                s+='Sensor'
                s+=reindent(str(sensor), 1) + "\n"       
        else:
            for plugin in self.plugins:
                s+='Plugin'
                s+=reindent(str(plugin), 1) + "\n"
        return s


################################################################################
######################### PLUGINS CLASSES ######################################
################################################################################
#FIXME missing __str__
class GenericPlugin(object):
    def __init__(self,name=None,filename=None,alwaysOn=None,updateRate=None):
        self.name = name
        self.filename = filename
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate
        self.plugintype=''


    @staticmethod
    def parse(node, verbose=True):
        for child in children(node):
            if child.localName == 'cameraName':
                plugtype = 'Camera'
                break
            elif child.localName == 'bumperTopicName':
                plugtype = 'Bumper'
                break
            elif child.localName == 'leftFrontJoint':
                plugtype = 'Drive'
                break
            elif child.localName == 'odometryTopic':
                plugtype = 'Odometry'
                break
            elif child.localName == 'height':
                plugtype = 'Video'
                break
            elif child.localName == 'mimicJoint':
                plugtype = 'Mimic'
                break
            elif child.localName == 'robotSimType':
                plugtype = 'Simu'
            else: 
                plugtype = 'IMU_LaserPlugin'
        print 'type of plugin : ' +str(plugtype)
        if plugtype=='Camera':
            plugin = CameraPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = CameraPlugin.parse(node,verbose)

        elif plugtype=='Bumper':
            plugin = BumperPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = BumperPlugin.parse(node,verbose)

        elif plugtype=='Drive':
            plugin = DrivePlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = DrivePlugin.parse(node,verbose)

        elif plugtype=='Odometry':
            plugin = OdometryPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = OdometryPlugin.parse(node,verbose)

        elif plugtype=='Video':
            plugin = VideoPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = VideoPlugin.parse(node,verbose)

        elif plugtype=='Mimic':
            plugin = MimicJointPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = MimicJointPlugin.parse(node,verbose)

        elif plugtype=='Simu':
            plugin = SimuPlugin(node.getAttribute('name'),node.getAttribute('filename'))
            plugin = SimuPlugin.parse(node,verbose)

        else:
      #      plugin = GenericPlugin() 
            plugin = IMU_LaserPlugin()
            plugin = IMU_LaserPlugin.parse(node,verbose)
        return plugin


    def to_xml(self, doc):
        xml = doc.createElement('plugin') 
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if self.plugintype=='Camera':
            xml = CameraPlugin.to_xml(doc)
        elif self.plugintype=='Bumper':
            xml = BumperPlugin.to_xml(doc)
        elif self.plugintype=='Drive':
            xml = DrivePlugin.to_xml(doc)
        elif self.plugintype=='Odometry':
            xml = OdometryPlugin.to_xml(doc)
        elif self.plugintype=='Video':
            xml = VideoPlugin.to_xml(doc)
        elif self.plugintype=='Mimic':
            xml = MimicJointPlugin.to_xml(doc)
        elif self.plugintype=='Simu':
            xml = SimuPlungin.to_xml(doc)    
        else:
            xml = IMU_LaserPlugin.to_xml(doc)
        return xml

class SimuPlugin(object):
    def __init__(self,name=None,filename=None,robotNamespace=None,robotSimType=None):
        self.name = name
        self.filename = filename
        self.robotNamespace = robotNamespace
        self.robotSimType = robotSimType

    @staticmethod
    def parse(node, verbose=True):
        plug = SimuPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'robotSimType':
                plug.robotSimType = str(child.childNodes[0].nodeValue)
            elif child.localName == 'robotNamespace':
                plug.robotNamespace = str(child.childNodes[0].nodeValue)
            
        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if(self.robotNamespace != None):
            xml.appendChild(create_element(doc, 'robotNamespace', self.robotNamespace))
        if(self.robotSimType != None):
            xml.appendChild(create_element(doc, 'robotSimType', self.robotSimType))
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if(self.robotNamespace != None):
            s += "RobotNamespace:\n"
            s += reindent(str(self.robotNamespace),1) + '\n'
        if(self.robotSimType != None):
            s += "RobotSimType:\n"
            s += reindent(str(self.robotSimType),1) + '\n'
        return s   


class MimicJointPlugin(object):
    def __init__(self,name=None,filename=None,joint=None,mimicjoint=None,multiplier=None,offset=None):
        self.name = name
        self.filename = filename
        self.joint = joint
        self.mimicjoint = mimicjoint
        self.multiplier = multiplier
        self.offset = offset

    @staticmethod
    def parse(node, verbose=True):
        plug = MimicJointPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'joint':
                plug.joint = str(child.childNodes[0].nodeValue)
            elif child.localName == 'mimicJoint':
                plug.mimicjoint = str(child.childNodes[0].nodeValue)
            elif child.localName == 'multiplier':
                plug.multiplier = str(child.childNodes[0].nodeValue)
            elif child.localName == 'offset':
                plug.offset = str(child.childNodes[0].nodeValue)
            
        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if(self.joint != None):
            xml.appendChild(create_element(doc, 'joint', self.joint))
        if(self.mimicjoint != None):
            xml.appendChild(create_element(doc, 'mimicjoint', self.mimicjoint))
        if(self.multiplier != None):
            xml.appendChild(create_element(doc, 'multiplier', self.multiplier))
        if(self.offset != None):
            xml.appendChild(create_element(doc, 'offset', self.offset))
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if(self.joint != None):
            s += "Joint:\n"
            s += reindent(str(self.joint),1) + '\n'
        if(self.mimicjoint != None):
            s += "Mimicjoint:\n"
            s += reindent(str(self.mimicjoint),1) + '\n'
        if(self.multiplier != None):
            s += "Multiplier:\n"
            s += reindent(str(self.multiplier),1) + '\n'
        if(self.offset != None):
            s += "Offset:\n"
            s += reindent(str(self.offset),1) + '\n'
        return s   

class BumperPlugin(object):
    def __init__(self,name=None,filename=None,bumperTopicName=None,frameName=None,alwaysOn=None,updateRate=None):
        self.name = name
        self.filename = filename
        self.bumperTopicName = bumperTopicName
        self.frameName = frameName
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate

    @staticmethod
    def parse(node, verbose=True):
        plug = BumperPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'bumperTopicName':
                plug.bumperTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'frameName':
                plug.frameName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'alwaysOn':
                plug.alwaysOn = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'updateRate':
                plug.updateRate = str(child.childNodes[0].nodeValue)
        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if(self.bumperTopicName != None):
            xml.appendChild(create_element(doc, 'bumperTopicName', self.bumperTopicName))
        if(self.frameName != None):
            xml.appendChild(create_element(doc, 'frameName', self.frameName))
        if(self.bumperTopicName != None):
            xml.appendChild(create_element(doc, 'alwaysOn', self.alwaysOn))
        if(self.updateRate != None):
            xml.appendChild(create_element(doc, 'updateRate', self.updateRate))
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if(self.bumperTopicName != None):
            s += "BumperTopicName:\n"
            s += reindent(str(self.bumperTopicName),1) + '\n'
        if(self.frameName != None):
            s += "FrameName:\n"
            s += reindent(str(self.frameName),1) + '\n'
        if(self.alwaysOn != None):
            s += "AlwaysOn:\n"
            s += reindent(str(self.alwaysOn),1) + '\n'
        if(self.updateRate != None):
            s += "UpdateRate:\n"
            s += reindent(str(self.updateRate),1) + '\n'
        return s   


class IMU_LaserPlugin(object):
    def __init__(self,name=None,filename=None,bodyName=None,robotNamespace=None,topicName=None,gaussianNoise=None,xyzOffset=None,rpyOffset=None,updateRate=None,alwaysOn=None, frameName=None, hokuyoMinIntensity=None, frameId=None, accelGaussianNoise=None,headingGaussianNoise=None,rateGaussianNoise=None):
        self.name = name
        self.filename = filename
        self.robotNamespace = robotNamespace
        self.topicName = topicName
        self.gaussianNoise = gaussianNoise
        self.xyzOffset = xyzOffset
        self.rpyOffset = rpyOffset
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate
        self.frameName = frameName
        self.hokuyoMinIntensity = hokuyoMinIntensity
        self.frameId = frameId
        self.accelGaussianNoise = accelGaussianNoise
        self.headingGaussianNoise = headingGaussianNoise
        self.rateGaussianNoise = rateGaussianNoise
        self.bodyName = bodyName

    @staticmethod
    def parse(node, verbose=True):
        plug = IMU_LaserPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'robotNamespace':
                plug.robotNamespace = str(child.childNodes[0].nodeValue)
            elif child.localName == 'topicName':
                plug.topicName = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'gaussianNoise':
                plug.gaussianNoise = str(child.childNodes[0].nodeValue)
            elif child.localName == 'xyzOffset':
                plug.xyzOffset = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            elif child.localName == 'rpyOffset':
                plug.rpyOffset = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            elif child.localName == 'alwaysOn':
                plug.alwaysOn = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'updateRate':
                plug.updateRate = str(child.childNodes[0].nodeValue)
            elif child.localName == 'frameName':
                plug.frameName = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'hokuyoMinIntensity':
                plug.hokuyoMinIntensity = str(child.childNodes[0].nodeValue)
            elif child.localName == 'frameId':
                plug.frameId = str(child.childNodes[0].nodeValue)
            elif child.localName == 'accelGaussianNoise':
                plug.accelGaussianNoise = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            elif child.localName == 'headingGaussianNoise':
                plug.headingGaussianNoise = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            elif child.localName == 'rateGaussianNoise':
                plug.rateGaussianNoise = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            elif child.localName == 'bodyName':
                plug.bodyName = str(child.childNodes[0].nodeValue)

        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if(self.robotNamespace != None):
            xml.appendChild(create_element(doc, 'robotNamespace', self.robotNamespace))
        if(self.topicName != None):
            xml.appendChild(create_element(doc, 'topicName', self.topicName))
        if(self.bodyName != None):
            xml.appendChild(create_element(doc, 'bodyName', self.bodyName)) 
        if(self.frameId != None):
            xml.appendChild(create_element(doc, 'frameId', self.frameId))  
        if(self.gaussianNoise != None):
            xml.appendChild(create_element(doc, 'gaussianNoise', self.gaussianNoise))
        if(self.accelGaussianNoise != None):
            xml.appendChild(create_element(doc, 'accelGaussianNoise', to_string(self.accelGaussianNoise)))
        if(self.headingGaussianNoise != None):
            xml.appendChild(create_element(doc, 'headingGaussianNoise', to_string(self.headingGaussianNoise)))
        if(self.rateGaussianNoise != None):
            xml.appendChild(create_element(doc, 'rateGaussianNoise', to_string(self.rateGaussianNoise)))
        if(self.xyzOffset != None):
            xml.appendChild(create_element(doc, 'xyzOffset', to_string(self.xyzOffset)))
        if(self.rpyOffset != None):
            xml.appendChild(create_element(doc, 'rpyOffset', to_string(self.rpyOffset)))
        if(self.alwaysOn != None):
            xml.appendChild(create_element(doc, 'alwaysOn', self.alwaysOn))
        if(self.updateRate != None):
            xml.appendChild(create_element(doc, 'updateRate', self.updateRate))       
        if(self.frameName != None):
            xml.appendChild(create_element(doc, 'frameName', self.frameName))
        if(self.hokuyoMinIntensity != None):
            xml.appendChild(create_element(doc, 'hokuyoMinIntensity', self.hokuyoMinIntensity))         
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if(self.robotNamespace != None):
            s += "RobotNamespace:\n"
            s += reindent(str(self.robotNamespace),1) + '\n'
        if(self.topicName != None):
            s += "TopicName:\n"
            s += reindent(str(self.topicName),1) + '\n'
        if(self.bodyName != None):
            s += "BodyName:\n"
            s += reindent(str(self.bodyName),1) + '\n'
        if(self.gaussianNoise != None):
            s += "GaussianNoise:\n"
            s += reindent(str(self.gaussianNoise),1) + '\n'
        if(self.xyzOffset != None):
            s += "XYZOffset\n"
            s += reindent(str(self.xyzOffset),1) + '\n'
        if(self.rpyOffset != None):
            s += "RPYOffset\n"
            s += reindent(str(self.rpyOffset),1) + '\n'
        if(self.alwaysOn != None):
            s += "AlwaysOn\n"
            s += reindent(str(self.alwaysOn),1) + '\n'
        if(self.updateRate != None):
            s += "UpdateRate:\n"
            s += reindent(str(self.updateRate),1) + '\n'
        if(self.frameName != None):
            s += "FrameName\n"
            s += reindent(str(self.frameName),1) + '\n'
        if(self.hokuyoMinIntensity != None):
            s += "HokuyoMinIntensity:\n"
            s += reindent(str(self.hokuyoMinIntensity),1) + '\n'
        if(self.frameId != None):
            s += "FrameId\n"
            s += reindent(str(self.frameId),1) + '\n'
        if(self.accelGaussianNoise != None):
            s += "AccelGaussianNoise:\n"
            s += reindent(str(self.accelGaussianNoise),1) + '\n'
        if(self.headingGaussianNoise != None):
            s += "HeadingGaussianNoise\n"
            s += reindent(str(self.headingGaussianNoise),1) + '\n'
        if(self.rateGaussianNoise != None):
            s += "RateGaussianNoise:\n"
            s += reindent(str(self.rateGaussianNoise),1) + '\n'       
        return s   

class VideoPlugin(object):
    def __init__(self,name=None,filename=None,topicName=None,height=None,width=None,updateRate=None,alwaysOn=None):
        self.name = name
        self.filename = filename
        self.topicName = topicName
        self.height = height
        self.width = width
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate

    @staticmethod
    def parse(node, verbose=True):
        plug = VideoPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'height':
                plug.height = str(child.childNodes[0].nodeValue)
            elif child.localName == 'topicName':
                plug.topicName = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'width':
                plug.width = str(child.childNodes[0].nodeValue)
            elif child.localName == 'alwaysOn':
                plug.alwaysOn = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'updateRate':
                plug.updateRate = str(child.childNodes[0].nodeValue)
        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if(self.topicName != None):
            xml.appendChild(create_element(doc, 'topicName', self.topicName))
        if(self.height != None):
            xml.appendChild(create_element(doc, 'height', self.height))
        if(self.width != None):
            xml.appendChild(create_element(doc, 'width', self.width))
        if(self.alwaysOn != None):
            xml.appendChild(create_element(doc, 'alwaysOn', self.alwaysOn))
        if(self.updateRate != None):
            xml.appendChild(create_element(doc, 'updateRate', self.updateRate))       
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if(self.topicName != None):
            s += "TopicName:\n"
            s += reindent(str(self.topicName),1) + '\n'
        if(self.height != None):
            s += "Height:\n"
            s += reindent(str(self.height),1) + '\n'
        if(self.width != None):
            s += "Width\n"
            s += reindent(str(self.width),1) + '\n'
        if(self.alwaysOn != None):
            s += "AlwaysOn\n"
            s += reindent(str(self.alwaysOn),1) + '\n'
        if(self.updateRate != None):
            s += "UpdateRate:\n"
            s += reindent(str(self.updateRate),1) + '\n'
        return s   

class OdometryPlugin(object):
    def __init__(self,name=None,filename=None,commandTopic=None,odometryTopic=None,odometryFrame=None,odometryRate=None,robotBaseFrame=None,updateRate=None,alwaysOn=None):
        self.name = name
        self.filename = filename
        self.commandTopic = commandTopic
        self.odometryTopic = odometryTopic
        self.odometryFrame = odometryFrame
        self.odometryRate = odometryRate
        self.robotBaseFrame = robotBaseFrame
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate

    @staticmethod
    def parse(node, verbose=True):
        plug = OdometryPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            if child.localName == 'commandTopic':
                plug.commandTopic = str(child.childNodes[0].nodeValue)
            elif child.localName == 'odometryTopic':
                plug.odometryTopic = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'odometryFrame':
                plug.odometryFrame = str(child.childNodes[0].nodeValue)
            elif child.localName == 'odometryRate':
                plug.odometryRate = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'robotBaseFrame':
                plug.robotBaseFrame = str(child.childNodes[0].nodeValue)
            elif child.localName == 'alwaysOn':
                plug.alwaysOn = str(child.childNodes[0].nodeValue)            
            elif child.localName == 'updateRate':
                plug.updateRate = str(child.childNodes[0].nodeValue)
        return plug
    
    def to_xml(self, doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if self.commandTopic != None :
            xml.appendChild(create_element(doc, 'commandTopic', self.commandTopic))
        if self.odometryTopic != None:
            xml.appendChild(create_element(doc, 'odometryTopic', self.odometryTopic))
        if self.odometryFrame != None :
            xml.appendChild(create_element(doc, 'odometryFrame', self.odometryFrame))
        if self.odometryRate != None :
            xml.appendChild(create_element(doc, 'odometryRate', self.odometryRate))
        if self.robotBaseFrame != None :
            xml.appendChild(create_element(doc, 'robotBaseFrame', self.robotBaseFrame))
        if self.alwaysOn != None :
            xml.appendChild(create_element(doc, 'alwaysOn', self.alwaysOn))
        if self.updateRate != None :
            xml.appendChild(create_element(doc, 'updateRate', self.updateRate))       
        return xml

    def __str__(self):
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if self.commandTopic != None:
            s += "CommandTopic:\n"
            s += reindent(str(self.commandTopic),1) + '\n'
        if self.odometryTopic != None:
            s += "OdometryTopic:\n"
            s += reindent(str(self.odometryTopic),1) + '\n'
        if self.odometryFrame != None :
            s += "OdometryFrame:\n"
            s += reindent(str(self.odometryFrame),1) + '\n'
        if self.odometryRate != None:
            s += "OdometryRate:\n"
            s += reindent(str(self.odometryRate),1) + '\n'
        if self.robotBaseFrame != None :
            s += "RobotBaseFrame\n"
            s += reindent(str(self.robotBaseFrame),1) + '\n'
        if self.alwaysOn != None :
            s += "AlwaysOn\n"
            s += reindent(str(self.alwaysOn),1) + '\n'
        if self.updateRate != None :
            s += "UpdateRate:\n"
            s += reindent(str(self.updateRate),1) + '\n'
        return s   

class CameraPlugin(object):
    def __init__(self,name=None,filename=None,alwaysOn=None,updateRate=None, cameraName=None, imageTopicName=None,cameraInfoTopicName=None,frameName=None,rightFrameName=None,hackBaseline=None,distortionK1=None,distortionK2=None,distortionK3=None,distortionT1=None,distortionT2=None,baseline=None,depthImageTopicName=None,depthImageInfoTopicName=None,pointCloudTopicName=None,pointCloudCutoff=None,CxPrime=None,Cx=None,Cy=None,focalLength=None,robotNamespace=None):
        self.name = name
        self.filename = filename
        self.alwaysOn = alwaysOn
        self.updateRate = updateRate
        self.cameraName = cameraName
        self.imageTopicName = imageTopicName
        self.cameraInfoTopicName =cameraInfoTopicName
        self.frameName = frameName
        self.rightFrameName = rightFrameName
        self.hackBaseline = hackBaseline
        self.distortionK1 = distortionK1
        self.distortionK2 = distortionK2
        self.distortionK3 = distortionK3
        self.distortionT1 = distortionT1
        self.distortionT2 = distortionT2
        self.baseline = baseline
        self.depthImageTopicName = depthImageTopicName
        self.depthImageInfoTopicName = depthImageInfoTopicName
        self.pointCloudTopicName = pointCloudTopicName
        self.pointCloudCutoff = pointCloudCutoff
        self.CxPrime = CxPrime
        self.Cx = Cx
        self.Cy = Cy
        self.robotNamespace = robotNamespace
        self.focalLength = focalLength

    @staticmethod
    def parse(node, verbose=True):
        plug = CameraPlugin()
        if node.hasAttribute('name'):
            plug.name = node.getAttribute('name')
        if node.hasAttribute('filename'):
            plug.filename = node.getAttribute('filename')
        for child in children(node):
            #print child.localName
            if child.localName == 'updateRate':
                plug.updateRate = str(child.childNodes[0].nodeValue)
            elif child.localName == 'alwaysOn':
                plug.alwaysOn = str(child.childNodes[0].nodeValue)
            elif child.localName == 'cameraName':
                plug.cameraName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'imageTopicName':
                plug.imageTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'cameraInfoTopicName':
                plug.cameraInfoTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'frameName':
                plug.frameName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'rightFrameName':
                plug.rightFrameName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'hackBaseline':
                plug.hackBaseline = str(child.childNodes[0].nodeValue)
            elif child.localName == 'distortionK1':
                plug.distortionK1 = str(child.childNodes[0].nodeValue)
            elif child.localName == 'distortionK2':
                plug.distortionK2 = str(child.childNodes[0].nodeValue)
            elif child.localName == 'distortionK3':
                plug.distortionK3 = str(child.childNodes[0].nodeValue)
            elif child.localName == 'distortionT1':
                plug.distortionT1 = str(child.childNodes[0].nodeValue)
            elif child.localName == 'distortionT2':
                plug.distortionT2 = str(child.childNodes[0].nodeValue)
            elif child.localName == 'baseline':
                plug.baseline = str(child.childNodes[0].nodeValue)
            elif child.localName == 'depthImageTopicName':
                plug.depthImageTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'depthImageInfoTopicName':
                plug.depthImageInfoTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'pointCloudTopicName':
                plug.pointCloudTopicName = str(child.childNodes[0].nodeValue)
            elif child.localName == 'pointCloudCutoff':
                plug.pointCloudCutoff = str(child.childNodes[0].nodeValue)
            elif child.localName == 'CxPrime':
                plug.CxPrime = str(child.childNodes[0].nodeValue)
            elif child.localName == 'Cx':
                plug.Cx = str(child.childNodes[0].nodeValue)
            elif child.localName == 'Cy':
                plug.Cy = str(child.childNodes[0].nodeValue)
            elif child.localName == 'robotNamespace':
                plug.robotNamespace = str(child.childNodes[0].nodeValue)
            elif child.localName == 'focalLength':
                plug.focalLength = str(child.childNodes[0].nodeValue)
        return plug

    def to_xml(self,doc):
        xml = doc.createElement('plugin')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.filename!= None:
            set_attribute(xml, 'filename', self.filename)
        if self.updateRate != None:
            xml.appendChild(create_element(doc, 'updateRate', self.updateRate))
        if self.alwaysOn != None:
            xml.appendChild(create_element(doc, 'alwaysOn', self.alwaysOn))
        if self.cameraName != None :
            xml.appendChild(create_element(doc, 'cameraName', self.cameraName))        
        if self.imageTopicName != None:
            xml.appendChild(create_element(doc, 'imageTopicName', self.imageTopicName))
        if self.cameraInfoTopicName != None:
            xml.appendChild(create_element(doc, 'cameraInfoTopicName', self.cameraInfoTopicName))
        if self.frameName != None:
            xml.appendChild(create_element(doc, 'frameName', self.frameName))
        if self.rightFrameName != None:
            xml.appendChild(create_element(doc, 'rightFrameName', self.rightFrameName))
        if self.hackBaseline != None :
            xml.appendChild(create_element(doc, 'hackBaseline', self.hackBaseline))   
        if self.distortionK1 != None :
            xml.appendChild(create_element(doc, 'distortionK1', self.distortionK1))
        if self.distortionK2 != None :
            xml.appendChild(create_element(doc, 'distortionK2', self.distortionK2))
        if self.distortionK3 != None :
            xml.appendChild(create_element(doc, 'distortionK3', self.distortionK3))
        if self.distortionT1 != None :
            xml.appendChild(create_element(doc, 'distortionT1', self.distortionT1))
        if self.distortionT2 != None :
            xml.appendChild(create_element(doc, 'distortionT2', self.distortionT2))        
        if self.baseline != None :
            xml.appendChild(create_element(doc, 'baseline', self.baseline))   
        if self.depthImageTopicName != None:
            xml.appendChild(create_element(doc, 'depthImageTopicName', self.depthImageTopicName))
        if self.depthImageInfoTopicName != None:
            xml.appendChild(create_element(doc, 'depthImageInfoTopicName', self.depthImageInfoTopicName))
        if self.pointCloudTopicName != None:
            xml.appendChild(create_element(doc, 'pointCloudTopicName', self.pointCloudTopicName))   
        if self.pointCloudCutoff != None :
            xml.appendChild(create_element(doc, 'pointCloudCutoff', self.pointCloudCutoff))
        if self.CxPrime != None :
            xml.appendChild(create_element(doc, 'CxPrime', self.CxPrime))
        if self.Cx != None :
            xml.appendChild(create_element(doc, 'Cx', self.Cx))
        if self.Cy != None :
            xml.appendChild(create_element(doc, 'Cy', self.Cy))
        if self.robotNamespace != None :
            xml.appendChild(create_element(doc, 'robotNamespace', self.robotNamespace))
        if self.focalLength != None :
            xml.appendChild(create_element(doc, 'focalLength', self.focalLength))

        return xml

    def __str__(self):
        s = ''
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.filename!= None:
            s += "Filename: {0}\n".format(self.filename)
        if self.updateRate != None :
            s += "UpdateRate:\n"
            s += reindent(str(self.updateRate),1) + '\n'
        if self.alwaysOn != None :
            s += "AlwaysOn:\n"
            s += reindent(str(self.alwaysOn),1) + '\n'
        if self.cameraName != None :
            s += "CameraName:\n"
            s += reindent(str(self.cameraName),1) + '\n'
        if self.imageTopicName != None :
            s += "ImageTopicName:\n"
            s += reindent(str(self.imageTopicName),1) + '\n'
        if self.cameraInfoTopicName != None :
            s += "CameraInfoTopicName:\n"
            s += reindent(str(self.cameraInfoTopicName),1) + '\n'
        if self.frameName != None:
            s += "FrameName:\n"
            s += reindent(str(self.frameName),1) + '\n'
        if self.rightFrameName != None:
            s += "RightFrameName:\n"
            s += reindent(str(self.rightFrameName),1) + '\n'
        if self.hackBaseline != None:
            s += "HackBaseline:\n"
            s += reindent(str(self.hackBaseline),1) + '\n'
        if self.distortionK1 != None:
            s += "DistortionK1:\n"
            s += reindent(str(self.distortionK1),1) + '\n'
        if self.distortionK2 != None:
            s += "DistortionK2:\n"
            s += reindent(str(self.distortionK2),1) + '\n'
        if self.distortionK3 != None :
            s += "DistortionK3:\n"
            s += reindent(str(self.distortionK3),1) + '\n'
        if self.distortionT1 != None :
            s += "DistortionT1:\n"
            s += reindent(str(self.distortionT1),1) + '\n'
        if self.distortionT2 != None:
            s += "DistortionT2:\n"
            s += reindent(str(self.distortionT2),1) + '\n'
        if self.baseline != None:
            s += "Baseline:\n"
            s += reindent(str(self.baseline),1) + '\n'
        if self.depthImageTopicName != None :
            s += "DepthImageTopicName:\n"
            s += reindent(str(self.depthImageTopicName),1) + '\n'
        if self.depthImageInfoTopicName != None :
            s += "DepthImageInfoTopicName:\n"
            s += reindent(str(self.depthImageInfoTopicName),1) + '\n'
        if self.pointCloudTopicName != None:
            s += "PointCloudTopicName:\n"
            s += reindent(str(self.pointCloudTopicName),1) + '\n'
        if self.pointCloudCutoff != None:
            s += "PointCloudCutoff:\n"
            s += reindent(str(self.pointCloudCutoff),1) + '\n'
        if self.CxPrime != None:
            s += "CxPrime:\n"
            s += reindent(str(self.CxPrime),1) + '\n'
        if self.Cx != None :
            s += "Cx:\n"
            s += reindent(str(self.Cx),1) + '\n'
        if self.Cy != None :
            s += "Cy:\n"
            s += reindent(str(self.Cy),1) + '\n'
        if self.robotNamespace != None :
            s += "RobotNamespace:\n"
            s += reindent(str(self.robotNamespace),1) + '\n'
        if self.focalLength != None :
            s += "FocalLength:\n"
            s += reindent(str(self.focalLength),1) + '\n'
        return s



   
#################################
###### SENSOR DECLARATION #######
#################################

class Sensor(object):
    def __init__(self,name=None,type=None,update_rate=None,camera=[],plugin=[],pose=None,visualize=None,ray=None):
        self.name = name
        self.type = type
        self.update_rate = update_rate
        self.cameras = camera
        self.plugins = plugin 
        self.pose = pose
        self.visualize = visualize
        self.ray = ray

    @staticmethod
    def parse(node, verbose=True):
        sens = Sensor(node.getAttribute('name'),node.getAttribute('type'))
        sens.plugins = []
        sens.cameras = []
        print 'parsing sensor ' + str(sens.name)
        for child in children(node):
            if child.localName == 'ray':
                sens.ray = Ray.parse(child,verbose)
            if child.localName == 'update_rate':
                sens.update_rate = str(child.childNodes[0].nodeValue)
            if child.localName == 'camera':
                sens.cameras.append(CameraSensor.parse(child,verbose))
            if child.localName == 'plugin':
                sens.plugins.append(GenericPlugin.parse(child,verbose))
            if child.localName == 'pose':
                sens.pose = array([float(x) for x in child.childNodes[0].nodeValue.split(' ')])
            if child.localName == 'visualize':
                sens.visualize = str(child.childNodes[0].nodeValue)
        return sens
    
    def to_xml(self, doc):
        xml = doc.createElement('sensor')
        set_attribute(xml, 'name', self.name)
        set_attribute(xml, 'type', self.type)
        if self.update_rate != None:
            xml.appendChild(create_element(doc, 'update_rate', self.update_rate))
        for camera in self.cameras:
            add( doc, xml, camera)
        for plugin in self.plugins:
            add(doc,xml,plugin)
        if self.pose != None:
            xml.appendChild(create_element(doc, 'pose', self.pose))
        #add(doc,xml,self.pose)
        if self.visualize != None:
            xml.appendChild(create_element(doc, 'visualize', self.visualize))
        if self.ray != None:
            add(doc,xml,self.ray)
        return xml
    
    def __str__(self):
        s = ""
        s += "Name: {0}\n".format(self.name)
        s += "Type: {0}\n".format(self.type)
        if self.update_rate != None:
            s += "Update_rate:\n"
            s += reindent(str(self.update_rate),1) + "\n"
        for camera in self.cameras:
            s+='Camera'
            s+=reindent(str(camera), 1) + "\n"
        for plugin in self.plugins:
            s+='Plugin'
            s+=reindent(str(plugin), 1) + "\n"
        
        if self.visualize != None:
            s += "Visualize:\n"
            s += reindent(str(self.visualize),1) + "\n"
        
        if self.ray != None:
            s += "Ray:\n"
            s += reindent(str(self.ray),1) + "\n"
        return s

class CameraSensor(object):
    def __init__(self,name=None,horizontal_fov=None,image=None,clip=None,noise=None):
        self.name = name
        self.horizontal_fov = horizontal_fov
        self.image = image
        self.clip = clip
        self.noise = noise

    @staticmethod
    def parse(node, verbose=True):
        sens = CameraSensor()
        if node.hasAttribute('name'):
            sens.name = node.getAttribute('name')
        for child in children(node):
            if child.localName == 'horizontal_fov':
                sens.horizontal_fov = str(child.childNodes[0].nodeValue)
            elif child.localName == 'image':
                sens.image = Image.parse(child,verbose)
            elif child.localName == 'clip':
                sens.clip = Clip.parse(child,verbose)
            elif child.localName == 'noise':
                sens.noise = Noise.parse(child,verbose)
        return sens
    
    def to_xml(self, doc):
        xml = doc.createElement('camera')
        if self.name!= None:
            set_attribute(xml, 'name', self.name)
        if self.horizontal_fov != None :
            xml.appendChild(create_element(doc, 'horizontal_fov', self.horizontal_fov))
        if self.image != None:
            add( doc, xml, self.image)
        if self.clip != None :
            add( doc, xml, self.clip)
        if self.noise != None :
            add( doc, xml, self.noise)
        return xml

    def __str__(self):
        s = ''
        if self.name!= None:
            s = "Name: {0}\n".format(self.name)
        if self.horizontal_fov != None:
            s += "Horizontal_fov:\n"
            s += reindent(str(self.horizontal_fov),1) + '\n'
        if self.image != None:
            s += "Image:\n"
            s += reindent(str(self.image),1) + '\n'
        if self.clip != None :
            s += "Clip:\n"
            s += reindent(str(self.clip),1) + '\n'
        if self.noise != None:
            s += "Noise:\n"
            s += reindent(str(self.noise),1) + '\n'
        return s   



################################
##### Auxiliary Elements #######
################################

class Clip(object):
    def __init__(self,near=None,far=None):
        self.near = near
        self.far = far

    @staticmethod
    def parse(node, verbose=True):
        clip = Clip()
        for child in children(node):
            if child.localName == 'near':
                clip.near = str(child.childNodes[0].nodeValue)
            elif child.localName == 'far':
                clip.far = str(child.childNodes[0].nodeValue)
        return clip
    
    def to_xml(self, doc):
        xml = doc.createElement('clip')
        if self.near != None :
            xml.appendChild(create_element(doc, 'near', self.near))
        if self.far != None:
            xml.appendChild(create_element(doc, 'far', self.far))
        return xml

    def __str__(self):
        s = ''
        if self.near != None:
            s += "Near:\n"
            s += reindent(str(self.near),1) + '\n'
        if self.far != None:
            s += "Far:\n"
            s += reindent(str(self.far),1) + '\n'
        return s   

class Image(object):
    def __init__(self,height=None,width=None,format=None):
        self.height = height
        self.width = width
        self.format = format

    @staticmethod
    def parse(node, verbose=True):
        img = Image()
        for child in children(node):
            if child.localName == 'height':
                img.height = str(child.childNodes[0].nodeValue)
            elif child.localName == 'width':
                img.width = str(child.childNodes[0].nodeValue)
            elif child.localName == 'format':
                img.format = str(child.childNodes[0].nodeValue)
        return img
    
    def to_xml(self, doc):
        xml = doc.createElement('image')
        if self.height != None :
            xml.appendChild(create_element(doc, 'height', self.height))
        if self.width != None :
            xml.appendChild(create_element(doc, 'width', self.width))
        if self.format != None:
            xml.appendChild(create_element(doc, 'format', self.format))
        return xml

    def __str__(self):
        s = ''
        if self.height != None:
            s += "Height:\n"
            s += reindent(str(self.height),1) + '\n'
        if self.width != None:
            s += "Width:\n"
            s += reindent(str(self.width),1) + '\n'
        if self.format != None:
            s += "Format:\n"
            s += reindent(str(self.format),1) + '\n'
        return s  


class Noise(object):
    def __init__(self,type=None,mean=None,stddev=None):
        self.type = type
        self.mean = mean
        self.stddev = stddev

    @staticmethod
    def parse(node, verbose=True):
        noise = Noise()
        for child in children(node):
            if child.localName == 'type':
                noise.type = str(child.childNodes[0].nodeValue)
            elif child.localName == 'mean':
                noise.mean = str(child.childNodes[0].nodeValue)
            elif child.localName == 'stddev':
                noise.stddev = str(child.childNodes[0].nodeValue)
        return noise
    
    def to_xml(self, doc):
        xml = doc.createElement('noise')
        if self.type != None :
            xml.appendChild(create_element(doc, 'type', self.type))
        if self.mean != None :
            xml.appendChild(create_element(doc, 'mean', self.mean))
        if self.stddev != None:
            xml.appendChild(create_element(doc, 'stddev', self.stddev))
        return xml

    def __str__(self):
        s = ''
        if self.type != None:
            s += "Type:\n"
            s += reindent(str(self.type),1) + '\n'
        if self.mean != None:
            s += "Mean:\n"
            s += reindent(str(self.mean),1) + '\n'
        if self.stddev != None:
            s += "Stddev:\n"
            s += reindent(str(self.stddev),1) + '\n'
        return s  


class Ray(object):
    def __init__(self,scan=None,range=None,noise=None):
        self.scan = scan
        self.range = range
        self.noise = noise

    @staticmethod
    def parse(node, verbose=True):
        ray = Ray()
        for child in children(node):
            if child.localName == 'scan':
                ray.scan = Scan.parse(child,verbose)# str(child.childNodes[0].nodeValue)
            elif child.localName == 'range':
                ray.range = Range.parse(child,verbose) #str(child.childNodes[0].nodeValue)
            elif child.localName == 'noise':
                ray.noise = Noise.parse(child,verbose)
        return ray
    
    def to_xml(self, doc):
        xml = doc.createElement('ray')
        if self.scan != None :
            add( doc, xml, self.scan)
        if self.range != None :
            add( doc, xml, self.range)
        if self.noise != None:
            add( doc, xml, self.noise)
        return xml

    def __str__(self):
        s = ''
        if self.scan != None:
            s += "Scan:\n"
            s += reindent(str(self.scan),1) + '\n'
        if self.range != None:
            s += "Range:\n"
            s += reindent(str(self.range),1) + '\n'
        if self.noise != None:
            s += "Noise:\n"
            s += reindent(str(self.noise),1) + '\n'
        return s  

class Range(object):
    def __init__(self,min=None,max=None,resolution=None):
        self.min = min 
        self.max = max
        self.resolution = resolution

    @staticmethod
    def parse(node, verbose=True):
        rang = Range()
        for child in children(node):
            if child.localName == 'min':
                rang.min = str(child.childNodes[0].nodeValue)
            elif child.localName == 'max':
                rang.max = str(child.childNodes[0].nodeValue)
            elif child.localName == 'resolution':
                rang.resolution = str(child.childNodes[0].nodeValue)
        return rang
    
    def to_xml(self, doc):
        xml = doc.createElement('range')
        if self.min != None :
            xml.appendChild(create_element(doc, 'min', self.min))   
        if self.max != None :
            xml.appendChild(create_element(doc, 'max', self.max))           
        if self.resolution != None:
            xml.appendChild(create_element(doc, 'resolution', self.resolution))      
        return xml

    def __str__(self):
        s = ''
        if self.min != None:
            s += "Min:\n"
            s += reindent(str(self.min),1) + '\n'
        if self.max != None:
            s += "Max:\n"
            s += reindent(str(self.max),1) + '\n'
        if self.resolution != None:
            s += "Resolution:\n"
            s += reindent(str(self.resolution),1) + '\n'
        return s  

class Scan(object):
    def __init__(self,horizontal=None):
        self.horizontal = horizontal 

    @staticmethod
    def parse(node, verbose=True):
        scan = Scan()
        for child in children(node):
            if child.localName == 'horizontal':
                scan.horizontal = Horizontal.parse(child,verbose)# str(child.childNodes[0].nodeValue)
        return scan
    
    def to_xml(self, doc):
        xml = doc.createElement('scan')
        if self.horizontal != None :
            add( doc, xml, self.horizontal)
        return xml

    def __str__(self):
        s = ''
        if self.horizontal != None:
            s += "Horizontal:\n"
            s += reindent(str(self.horizontal),1) + '\n'
        return s 


class Horizontal(object):
    def __init__(self,samples=None,resolution=None,min_angle=None,max_angle=None):
        self.samples = samples  
        self.resolution =  resolution 
        self.min_angle = min_angle  
        self.max_angle = max_angle 

    @staticmethod
    def parse(node, verbose=True):
        hori = Horizontal()
        for child in children(node):
            if child.localName == 'samples':
                hori.samples = str(child.childNodes[0].nodeValue)
            elif child.localName == 'resolution':
                hori.resolution = str(child.childNodes[0].nodeValue)
            elif child.localName == 'min_angle':
                hori.min_angle = str(child.childNodes[0].nodeValue)
#Noise.parse(child,verbose)
            elif child.localName == 'max_angle':
                hori.max_angle = str(child.childNodes[0].nodeValue)
        return hori
    
    def to_xml(self, doc):
        xml = doc.createElement('horizontal')
        if self.samples != None :
            xml.appendChild(create_element(doc, 'samples', self.samples))
        if self.resolution != None :
            xml.appendChild(create_element(doc, 'resolution', self.resolution))
        if self.min_angle != None :
            xml.appendChild(create_element(doc, 'min_angle', self.min_angle))
        if self.max_angle != None :
            xml.appendChild(create_element(doc, 'max_angle', self.samples))

        return xml

    def __str__(self):
        s = ''
        if self.samples != None:
            s += "Samples:\n"
            s += reindent(str(self.samples),1) + '\n'
        if self.resolution != None:
            s += "Resolution:\n"
            s += reindent(str(self.resolution),1) + '\n'
        if self.min_angle != None:
            s += "Min_angle:\n"
            s += reindent(str(self.min_angle),1) + '\n'
        if self.max_angle != None:
            s += "Max_angle:\n"
            s += reindent(str(self.max_angle),1) + '\n'
        return s 


