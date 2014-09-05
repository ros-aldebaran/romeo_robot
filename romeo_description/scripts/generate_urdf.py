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

# This file takes the official URDF aldebaran files and convert them
# to rep120 compliant urdf files. It also includes the meshes from nao_meshes package 
# allowing to display the model in RVIZ

# authors: Mikael Arguedas [mikael DOT arguedas AT gmail DOT com]
#FIXME Add : Fingers for Humanoids 
#FIXME Fix bug regarding Feet and sole in gazebo

import sys
import argparse
from romeo_description.urdf import URDF
import romeo_description.gazeboUrdf
import romeo_description.urdf as ur
import romeo_description.nao_dictionaries as dico
import subprocess
import os
import math
from xml.dom.minidom import Document

dicoNaoXacro= {
'head':'gaze',
'legs':'sole',
'arms':'gripper',
'torso':'torso',
}

dicoRomeoXacro= {
'head':'gaze',
'legs':'sole',
'arms':'wrist',
'torso':'body',
}

dicoPepperXacro = {
'head':'Head',
'legs':'base_footprint',
'arms':'wrist',
'torso':'torso',

}

parser = argparse.ArgumentParser(usage='Load an URDF file')
parser.add_argument('-i','--input', default=None, help='URDF file to load')
parser.add_argument('-m','--meshes', choices=['cylinder','dae'], 
default='dae', 
help='Choose the meshes to use: geometricalShapes, or collada(.dae)')
parser.add_argument('-r','--REP120', choices=['true','false'], default='true',
help='Rename the links to be REP120 compliant')
parser.add_argument('-x','--xacro', choices=['urdf','robot'],
default='robot', help='Chose robot part to generate. choosing urdf create a single urdf file with the entire robot. robot will output a xacro file for every kinematic chain on the robot')

####################
##### FUNCTIONS ####
####################

def defineMaterials(robot):
    robot.add_material(ur.Material('Black',ur.Color(0.1,0.1,0.1,1)))
    robot.add_material(ur.Material('LightGrey',ur.Color(0.9,0.9,0.9,1)))
    robot.add_material(ur.Material('Grey',ur.Color(0.6,0.6,0.6,1)))
    robot.add_material(ur.Material('DarkGrey',ur.Color(0.3,0.3,0.3,1)))


def REP120func(robot):
    global name
    global meshversion
    global version
    global dicLinks
    print 'renaming joints_links and links to comply to REP120'
    #adding base_link and gaze link
    robot.add_link(ur.Link('base_link'))
    robot.add_joint(ur.Joint('base_joint', 'base_link', 'torso', 'fixed',
        None, ur.Pose((0,0,0),(0,0,0))))

#    robot.add_link(ur.Link('l_sole'))
#    robot.add_link(ur.Link('r_sole'))
    robot.add_link(ur.Link('gaze'))
    robot.add_link(ur.Link('l_gripper',None,None,ur.Inertial(1.1e-9,0,0,1.1e-9,0,1.1e-9,2e-6,ur.Pose((0,0,0),(0,0,0)))))
    robot.add_link(ur.Link('r_gripper',None,None,ur.Inertial(1.1e-9,0,0,1.1e-9,0,1.1e-9,2e-6,ur.Pose((0,0,0),(0,0,0)))))

# this script is only for nao from now on


    for joint in robot.joints:
        try:
            robot.joints[joint].parent = dicLinks[robot.joints[joint].parent]
        except KeyError:
            pass
        try:
            robot.joints[joint].child = dicLinks[robot.joints[joint].child]
        except KeyError:
            pass
    for link in robot.links.keys():
        try:
            robot.rename_link(link, dicLinks[link])
        except KeyError:
            pass
            
        except ValueError:
            pass
    
## Used for Gazebo but we don't use it right now     
#    for link in robot.links.keys():  
#        try:
#        robot.add_gazebo(ur.Gazebo(robot.links[link].name,None,"false",None,None,None,0.5,0.5,None,None,None,"false"))
#        except KeyError:
#            pass 


### SENSORS ###
    if name == 'nao':
        print 'adding nao missing joints and links'
        robot.add_link(ur.Link('CameraTop_frame'))
        robot.add_link(ur.Link('CameraBottom_frame'))
        # add joints accordingly
        ## Strange behaviour in gazebo with Feet and sole
#        robot.add_joint(ur.Joint('l_sole_joint','LFeet','l_sole','fixed', \
#        None,ur.Pose((0,0,-dico.Nao_offsets['FootHeight_' + version]),(0,0,0))))
#        robot.add_joint(ur.Joint('r_sole_joint','RFeet','r_sole','fixed', \
#        None,ur.Pose((0,0,-dico.Nao_offsets['FootHeight_' + version]),(0,0,0))))

    # Define mesh version to include to xacro files 
        if version == 'V32':
            meshversion = version
        elif version == 'V33' or version == 'V40' or version == 'V50':
            meshversion = 'V40'

        # Check if Head is V3 or V4
        if version == 'V32' or version == 'V33' :
            robot.add_joint(ur.Joint('CameraTop_joint', 'Head',
            'CameraTop_frame', 'fixed', None, ur.Pose((dicOffsets['CameraTopV3OffsetX'],\
            dicOffsets['CameraTopV3OffsetY'],dicOffsets['CameraTopV3OffsetZ']),\
            (dicOffsets['CameraTopV3RotX'],dicOffsets['CameraTopV3RotY'],dicOffsets['CameraTopV3RotZ']))))
            robot.add_joint(ur.Joint('CameraBottom_joint', 'Head',
            'CameraBottom_frame', 'fixed', None,
            ur.Pose((dicOffsets['CameraBottomV3OffsetX'],dicOffsets['CameraBottomV3OffsetY'],dicOffsets['CameraBottomV3OffsetZ']),\
            (dicOffsets['CameraBottomV3RotX'],dicOffsets['CameraBottomV3RotY'],dicOffsets['CameraBottomV3RotZ']))))
            robot.add_joint(ur.Joint('gaze_int', 'Head',
            'gaze', 'fixed', None, ur.Pose((dicOffsets['CameraTopV3OffsetX'],\
            dicOffsets['CameraTopV3OffsetY'],dicOffsets['CameraTopV3OffsetZ']),\
            (dicOffsets['CameraTopV3RotX'],dicOffsets['CameraTopV3RotY'],dicOffsets['CameraTopV3RotZ']))))

        elif version =='V40' or version =='V50':
            robot.add_joint(ur.Joint('CameraTop_joint', 'Head',
            'CameraTop_frame', 'fixed', None, \
            ur.Pose((dicOffsets['CameraTopV4OffsetX'],dicOffsets['CameraTopV4OffsetY'],dicOffsets['CameraTopV4OffsetZ']),\
            (dicOffsets['CameraTopV4RotX'],dicOffsets['CameraTopV4RotY'],dicOffsets['CameraTopV4RotZ']))))
            robot.add_joint(ur.Joint('CameraBottom_joint', 'Head',
            'CameraBottom_frame', 'fixed', None,
            ur.Pose((dicOffsets['CameraBottomV4OffsetX'],dicOffsets['CameraBottomV4OffsetY'],dicOffsets['CameraBottomV4OffsetZ']),\
            (dicOffsets['CameraBottomV4RotX'],dicOffsets['CameraBottomV4RotY'],dicOffsets['CameraBottomV4RotZ']))))
            robot.add_joint(ur.Joint('gaze_joint', 'Head',
            'gaze', 'fixed', None, \
            ur.Pose((dicOffsets['CameraTopV4OffsetX'],dicOffsets['CameraTopV4OffsetY'],dicOffsets['CameraTopV4OffsetZ']),\
            (dicOffsets['CameraTopV4RotX'],dicOffsets['CameraTopV4RotY'],dicOffsets['CameraTopV4RotZ']))))

        #sonars
        robot.add_link(ur.Link('LSonar_frame'))
        robot.add_link(ur.Link('RSonar_frame'))
        robot.add_joint(ur.Joint('LSonar_joint', 'torso',
        'LSonar_frame', 'fixed', None, ur.Pose((dicOffsets['SonarOffsetX'],dicOffsets['SonarOffsetY'],dicOffsets['SonarOffsetZ']),(dicOffsets['SonarRotX'],dicOffsets['SonarRotY'],-dicOffsets['SonarRotZ']))))
        robot.add_joint(ur.Joint('RSonar_joint', 'torso',
        'RSonar_frame', 'fixed', None, ur.Pose((dicOffsets['SonarOffsetX'],-dicOffsets['SonarOffsetY'],dicOffsets['SonarOffsetZ']),(dicOffsets['SonarRotX'],dicOffsets['SonarRotY'],dicOffsets['SonarRotZ']))))

        # Infrared Sensors
        robot.add_link(ur.Link('LInfraRed_frame'))
        robot.add_link(ur.Link('RInfraRed_frame'))
            # Attached to HeadPitchLink = Head
        robot.add_joint(ur.Joint('LInfraRed_joint', 'Head',
        'LInfraRed_frame', 'fixed', None, ur.Pose((dicOffsets['IROffsetX'],dicOffsets['IROffsetY'],dicOffsets['IROffsetZ']),(0,0,0))))
        robot.add_joint(ur.Joint('RInfraRed_joint', 'Head',
        'RInfraRed_frame', 'fixed', None, ur.Pose((dicOffsets['IROffsetX'],-dicOffsets['IROffsetY'],dicOffsets['IROffsetZ']),(0,0,0))))
        
        # FSRs
        robot.add_link(ur.Link('LFsrFL_frame'))
        robot.add_link(ur.Link('RFsrFL_frame'))
        robot.add_link(ur.Link('LFsrFR_frame'))
        robot.add_link(ur.Link('RFsrFR_frame'))
        robot.add_link(ur.Link('LFsrRL_frame'))
        robot.add_link(ur.Link('RFsrRL_frame'))
        robot.add_link(ur.Link('LFsrRR_frame'))
        robot.add_link(ur.Link('RFsrRR_frame'))
        
        # Attached to LAnklePitchLink = l_ankle
        robot.add_joint(ur.Joint('LFsrFL_joint', 'l_ankle',
        'LFsrFL_frame', 'fixed', None, ur.Pose((dicOffsets['FSROffsetXFL'],dicOffsets['FSROffsetYFL'],0.0),(0,0,0))))
        robot.add_joint(ur.Joint('RFsrFL_joint', 'r_ankle',
        'RFsrFL_frame', 'fixed', None, ur.Pose((dicOffsets['FSROffsetXFL'],dicOffsets['FSROffsetYFR'],0.0),(0,0,0))))

        robot.add_joint(ur.Joint('LFsrFR_joint', 'l_ankle',
        'LFsrFR_frame', 'fixed', None, ur.Pose((dicOffsets['FSROffsetXFL'],-dicOffsets['FSROffsetYFR'],0.0),(0,0,0))))
        robot.add_joint(ur.Joint('RFsrFR_joint', 'r_ankle',
        'RFsrFR_frame', 'fixed', None, ur.Pose((dicOffsets['FSROffsetXFL'],-dicOffsets['FSROffsetYFL'],0.0),(0,0,0))))

        robot.add_joint(ur.Joint('LFsrRL_joint', 'l_ankle',
        'LFsrRL_frame', 'fixed', None, ur.Pose((-dicOffsets['FSROffsetXRL'],dicOffsets['FSROffsetYFL'],0.0),(0,0,0))))
        robot.add_joint(ur.Joint('RFsrRL_joint', 'r_ankle',
        'RFsrRL_frame', 'fixed', None, ur.Pose((-dicOffsets['FSROffsetXRL'],dicOffsets['FSROffsetYRR'],0.0),(0,0,0))))

        robot.add_joint(ur.Joint('LFsrRR_joint', 'l_ankle',
        'LFsrRR_frame', 'fixed', None, ur.Pose((-dicOffsets['FSROffsetXRR'],-dicOffsets['FSROffsetYRR'],0.0),(0,0,0))))
        robot.add_joint(ur.Joint('RFsrRR_joint', 'r_ankle',
        'RFsrRR_frame', 'fixed', None, ur.Pose((-dicOffsets['FSROffsetXRR'],-dicOffsets['FSROffsetYFL'],0.0),(0,0,0))))

        # IMU
            # Attached to TorsoLink = torso
        robot.add_link(ur.Link('ImuAccelerometer_frame'))
        robot.add_link(ur.Link('ImuGyrometer_frame'))
        robot.add_joint(ur.Joint('ImuAccelerometer_joint', 'torso',
        'ImuAccelerometer_frame', 'fixed', None, ur.Pose((-dicOffsets['IMUAcceleroOffsetX'],dicOffsets['IMUAcceleroOffsetY'],dicOffsets['IMUAcceleroOffsetZ']),(0,0,0))))
        robot.add_joint(ur.Joint('ImuGyrometer_joint', 'torso',
        'ImuGyrometer_frame', 'fixed', None, ur.Pose((-dicOffsets['IMUGyroOffsetX'],-dicOffsets['IMUGyroOffsetY'],dicOffsets['IMUGyroOffsetZ']),(0,0,0))))
       
        # Bumpers
        robot.add_link(ur.Link('LFootBumperLeft_frame'))
        robot.add_link(ur.Link('LFootBumperRight_frame'))
        robot.add_link(ur.Link('RFootBumperLeft_frame'))
        robot.add_link(ur.Link('RFootBumperRight_frame'))
            # Attached to LAnklePitchLink = l_ankle
        robot.add_joint(ur.Joint('LFootBumperLeft_joint', 'l_ankle',
        'LFootBumperLeft_frame', 'fixed', None, ur.Pose((dicOffsets['FootBumperOffsetX'],dicOffsets['FootBumperOffsetYLeft'],-dicOffsets['FootBumperOffsetZ']),(0,0,0))))
        robot.add_joint(ur.Joint('LFootBumperRight_joint', 'l_ankle',
        'LFootBumperRight_frame', 'fixed', None, ur.Pose((dicOffsets['FootBumperOffsetX'],-dicOffsets['FootBumperOffsetYRight'],-dicOffsets['FootBumperOffsetZ']),(0,0,0))))
        robot.add_joint(ur.Joint('RFootBumperLeft_joint', 'r_ankle',
        'RFootBumperLeft_frame', 'fixed', None, ur.Pose((dicOffsets['FootBumperOffsetX'],dicOffsets['FootBumperOffsetYRight'],-dicOffsets['FootBumperOffsetZ']),(0,0,0))))
        robot.add_joint(ur.Joint('RFootBumperRight_joint', 'r_ankle',
        'RFootBumperRight_frame', 'fixed', None, ur.Pose((dicOffsets['FootBumperOffsetX'],-dicOffsets['FootBumperOffsetYLeft'],-dicOffsets['FootBumperOffsetZ']),(0,0,0))))

        # Chest Button
        robot.add_link(ur.Link('ChestButton_frame'))
            # Attached to TorsoLink = torso
        robot.add_joint(ur.Joint('ChestButton_joint', 'torso',
        'ChestButton_frame', 'fixed', None, ur.Pose((dicOffsets['ChestButtonOffsetX'],dicOffsets['ChestButtonOffsetY'],dicOffsets['ChestButtonOffsetZ']),(0,0,0))))

 


        if version != 'V32':
            # Touch sensors
                # Head Touch sensors 
                # Attached to HeadPitchLink = Head
            robot.add_link(ur.Link('HeadTouchFront_frame'))
            robot.add_link(ur.Link('HeadTouchMiddle_frame'))
            robot.add_link(ur.Link('HeadTouchRear_frame'))

            robot.add_joint(ur.Joint('HeadTouchFront_joint', 'Head',
            'HeadTouchFront_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchFrontOffsetX'],dicOffsets['HeadTouchFrontOffsetY'],dicOffsets['HeadTouchFrontOffsetZ']),(dicOffsets['HeadTouchFrontRotX'],dicOffsets['HeadTouchFrontRotY'],dicOffsets['HeadTouchFrontRotZ']))))

            robot.add_joint(ur.Joint('HeadTouchMiddle_joint', 'Head',
            'HeadTouchMiddle_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchMiddleOffsetX'],dicOffsets['HeadTouchMiddleOffsetY'],dicOffsets['HeadTouchMiddleOffsetZ']),(dicOffsets['HeadTouchMiddleRotX'],dicOffsets['HeadTouchMiddleRotY'],dicOffsets['HeadTouchMiddleRotZ']))))

            robot.add_joint(ur.Joint('HeadTouchRear_joint', 'Head',
            'HeadTouchRear_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchRearOffsetX'],dicOffsets['HeadTouchRearOffsetY'],dicOffsets['HeadTouchRearOffsetZ']),(dicOffsets['HeadTouchRearRotX'],dicOffsets['HeadTouchRearRotY'],dicOffsets['HeadTouchRearRotZ']))))
                

                # LHand touch sensors
                # Attached to LWristYawLink = l_wrist

            robot.add_link(ur.Link('LHandTouchLeft_frame'))
            robot.add_link(ur.Link('LHandTouchBack_frame'))
            robot.add_link(ur.Link('LHandTouchRight_frame'))
  
            robot.add_joint(ur.Joint('LHandTouchLeft_joint', 'l_wrist',
            'LHandTouchLeft_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchLeftOffsetX'],dicOffsets['HandTouchLeftOffsetY'],dicOffsets['HandTouchLeftOffsetZ']),(dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX']))))
            robot.add_joint(ur.Joint('LHandTouchBack_joint', 'l_wrist',
            'LHandTouchBack_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchBackOffsetX'],dicOffsets['HandTouchBackOffsetY'],dicOffsets['HandTouchBackOffsetZ']),(dicOffsets['HandTouchBackRotX'],dicOffsets['HandTouchBackRotY'],dicOffsets['HandTouchBackRotZ']))))
            robot.add_joint(ur.Joint('LHandTouchRight_joint', 'l_wrist',
            'LHandTouchRight_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchRightOffsetX'],dicOffsets['HandTouchRightOffsetY'],dicOffsets['HandTouchRightOffsetZ']),(dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX'],-dicOffsets['HandTouchLeftRotX']))))

                # RHand touch sensors
            robot.add_link(ur.Link('RHandTouchLeft_frame'))
            robot.add_link(ur.Link('RHandTouchBack_frame'))
            robot.add_link(ur.Link('RHandTouchRight_frame'))

            robot.add_joint(ur.Joint('RHandTouchLeft_joint', 'r_wrist',
            'RHandTouchLeft_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchLeftOffsetX'],dicOffsets['HandTouchLeftOffsetY'],dicOffsets['HandTouchLeftOffsetZ']),(dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX']))))
            robot.add_joint(ur.Joint('RHandTouchBack_joint', 'r_wrist',
            'RHandTouchBack_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchBackOffsetX'],dicOffsets['HandTouchBackOffsetY'],dicOffsets['HandTouchBackOffsetZ']),(dicOffsets['HandTouchBackRotX'],dicOffsets['HandTouchBackRotY'],dicOffsets['HandTouchBackRotZ']))))
            robot.add_joint(ur.Joint('RHandTouchRight_joint', 'r_wrist',
            'RHandTouchRight_frame', 'fixed', None, ur.Pose((dicOffsets['HandTouchRightOffsetX'],dicOffsets['HandTouchRightOffsetY'],dicOffsets['HandTouchRightOffsetZ']),(dicOffsets['HandTouchLeftRotX'],dicOffsets['HandTouchLeftRotX'],-dicOffsets['HandTouchLeftRotX']))))

        # no dictionnary for now because there's no use for now waiting for getting these values from documentation
        robot.add_joint(ur.Joint('LHand','l_wrist','l_gripper',\
        'revolute',None,ur.Pose((dicOffsets['HandOffsetX_' + version],0, \
        -dicOffsets['HandOffsetZ_' + version]),(0,0,0)),ur.JointLimit(0.177576,3,0.0,1.0)))
        robot.add_joint(ur.Joint('RHand','r_wrist','r_gripper', \
        'revolute',None,ur.Pose((dicOffsets['HandOffsetX_' + version],0, \
        -dicOffsets['HandOffsetZ_' + version]),(0,0,0)),ur.JointLimit(0.177576,3,0.0,1.0)))

    elif name=='romeo':
        #TODO add toe frames
        #TODO add gripper

        meshversion='V1'
        #################
        #### REP 120 ####
        #################
        robot.add_link(ur.Link('l_sole'))
        robot.add_link(ur.Link('r_sole'))
#        robot.add_link(ur.Link('l_toe'))
#        robot.add_link(ur.Link('r_toe'))
#        robot.add_link(ur.Link('l_gripper'))
#        robot.add_link(ur.Link('r_gripper'))

        robot.add_joint(ur.Joint('l_sole_joint', 'l_ankle',
        'l_sole', 'fixed', None, ur.Pose((0,0,-dicOffsets['FootHeight']),(0,0,0))))
        robot.add_joint(ur.Joint('r_sole_joint', 'r_ankle',
        'r_sole', 'fixed', None, ur.Pose((0,0,-dicOffsets['FootHeight']),(0,0,0))))
       
        robot.add_joint(ur.Joint('gaze_joint', 'HeadRollLink',
        'gaze', 'fixed', None, ur.Pose((dicOffsets['CameraLeftEyeOffsetX'],0,dicOffsets['CameraLeftEyeOffsetZ']),(0,0,0))))
 
        #################
        #### SENSORS ####
        #################
        ## IMUs
        robot.add_link(ur.Link('ImuTorsoAccelerometer_frame'))
        robot.add_link(ur.Link('ImuTorsoGyrometer_frame'))
        robot.add_link(ur.Link('ImuHeadAccelerometer_frame'))
        robot.add_link(ur.Link('ImuHeadGyrometer_frame'))

        robot.add_joint(ur.Joint('ImuTorsoAccelerometer_joint', 'body',
        'ImuTorsoAccelerometer_frame', 'fixed', None, ur.Pose((dicOffsets['IMUTorsoAcceleroOffsetX'],dicOffsets['IMUTorsoAcceleroOffsetY'],dicOffsets['IMUTorsoAcceleroOffsetZ']),(dicOffsets['IMUTorsoAcceleroRotX'],dicOffsets['IMUTorsoAcceleroRotY'],dicOffsets['IMUTorsoAcceleroRotZ']))))
        robot.add_joint(ur.Joint('ImuTorsoGyrometer_joint', 'body',
        'ImuTorsoGyrometer_frame', 'fixed', None, ur.Pose((dicOffsets['IMUTorsoGyroOffsetX'],dicOffsets['IMUTorsoGyroOffsetY'],dicOffsets['IMUTorsoGyroOffsetZ']),(dicOffsets['IMUTorsoGyroRotX'],dicOffsets['IMUTorsoGyroRotY'],dicOffsets['IMUTorsoGyroRotZ']))))

        robot.add_joint(ur.Joint('ImuHeadAccelerometer_joint', 'HeadRollLink',
        'ImuHeadAccelerometer_frame', 'fixed', None, ur.Pose((dicOffsets['IMUHeadAcceleroOffsetX'],dicOffsets['IMUHeadAcceleroOffsetY'],dicOffsets['IMUHeadAcceleroOffsetZ']),(dicOffsets['IMUHeadAcceleroRotX'],dicOffsets['IMUHeadAcceleroRotY'],dicOffsets['IMUHeadAcceleroRotZ']))))
        robot.add_joint(ur.Joint('ImuHeadGyrometer_joint', 'HeadRollLink',
        'ImuHeadGyrometer_frame', 'fixed', None, ur.Pose((dicOffsets['IMUHeadGyroOffsetX'],dicOffsets['IMUHeadGyroOffsetY'],dicOffsets['IMUHeadGyroOffsetZ']),(dicOffsets['IMUHeadGyroRotX'],dicOffsets['IMUHeadGyroRotY'],dicOffsets['IMUHeadGyroRotZ']))))

        ## Cameras
        robot.add_link(ur.Link('CameraLeftEye_frame'))
        robot.add_link(ur.Link('CameraRightEye_frame'))
        robot.add_link(ur.Link('CameraLeft_frame'))
        robot.add_link(ur.Link('CameraRight_frame'))
        robot.add_link(ur.Link('CameraDepth_frame'))


        robot.add_joint(ur.Joint('CameraLeftEye_joint', 'HeadRollLink',
        'CameraLeftEye_frame', 'fixed', None, ur.Pose((dicOffsets['CameraLeftEyeOffsetX'],\
        dicOffsets['CameraLeftEyeOffsetY'],dicOffsets['CameraLeftEyeOffsetZ']),\
        (dicOffsets['CameraLeftEyeRotX'],dicOffsets['CameraLeftEyeRotY'],dicOffsets['CameraLeftEyeRotZ']))))
        robot.add_joint(ur.Joint('CameraRightEye_joint', 'HeadRollLink',
        'CameraRightEye_frame', 'fixed', None, ur.Pose((dicOffsets['CameraRightEyeOffsetX'],\
        dicOffsets['CameraRightEyeOffsetY'],dicOffsets['CameraRightEyeOffsetZ']),\
        (dicOffsets['CameraRightEyeRotX'],dicOffsets['CameraRightEyeRotY'],dicOffsets['CameraRightEyeRotZ']))))


        robot.add_joint(ur.Joint('CameraLeft_joint', 'HeadRollLink',
        'CameraLeft_frame', 'fixed', None, ur.Pose((dicOffsets['CameraLeftOffsetX'],\
        dicOffsets['CameraLeftOffsetY'],dicOffsets['CameraLeftOffsetZ']),\
        (dicOffsets['CameraLeftRotX'],dicOffsets['CameraLeftRotY'],dicOffsets['CameraLeftRotZ']))))
        robot.add_joint(ur.Joint('CameraRight_joint', 'HeadRollLink',
        'CameraRight_frame', 'fixed', None, ur.Pose((dicOffsets['CameraRightOffsetX'],\
        dicOffsets['CameraRightOffsetY'],dicOffsets['CameraRightOffsetZ']),\
        (dicOffsets['CameraRightRotX'],dicOffsets['CameraRightRotY'],dicOffsets['CameraRightRotZ']))))

        robot.add_joint(ur.Joint('CameraDepth_joint', 'HeadRollLink',
        'CameraDepth_frame', 'fixed', None, ur.Pose((dicOffsets['CameraDepthOffsetX'],\
        dicOffsets['CameraDepthOffsetY'],dicOffsets['CameraDepthOffsetZ']),\
        (dicOffsets['CameraDepthRotX'],dicOffsets['CameraDepthRotY'],dicOffsets['CameraDepthRotZ']))))


        ## Foot Force Sensors

        robot.add_link(ur.Link('LFsrFL_frame'))
        robot.add_link(ur.Link('RFsrFL_frame'))
        robot.add_link(ur.Link('LFsrFR_frame'))
        robot.add_link(ur.Link('RFsrFR_frame'))
        robot.add_link(ur.Link('LFsrCenter_frame'))
        robot.add_link(ur.Link('RFsrCenter_frame'))
        robot.add_link(ur.Link('LFsrRCenter_frame'))
        robot.add_link(ur.Link('RFsrRCenter_frame'))

        robot.add_joint(ur.Joint('LFsrFL_joint', 'l_ankle',
        'LFsrFL_frame', 'fixed', None, ur.Pose((dicOffsets['FsrFLOffsetX'],\
        dicOffsets['FsrFLOffsetY'],dicOffsets['FsrFLOffsetZ']),\
        (dicOffsets['FsrFLRotX'],dicOffsets['FsrFLRotY'],dicOffsets['FsrFLRotZ']))))
        robot.add_joint(ur.Joint('RFsrFL_joint', 'r_ankle',
        'RFsrFL_frame', 'fixed', None, ur.Pose((dicOffsets['FsrFLOffsetX'],\
        dicOffsets['FsrFLOffsetY'],dicOffsets['FsrFLOffsetZ']),\
        (dicOffsets['FsrFLRotX'],dicOffsets['FsrFLRotY'],dicOffsets['FsrFLRotZ']))))

        robot.add_joint(ur.Joint('LFsrFR_joint', 'l_ankle',
        'LFsrFR_frame', 'fixed', None, ur.Pose((dicOffsets['FsrFROffsetX'],\
        dicOffsets['FsrFROffsetY'],dicOffsets['FsrFROffsetZ']),\
        (dicOffsets['FsrFRRotX'],dicOffsets['FsrFRRotY'],dicOffsets['FsrFRRotZ']))))
        robot.add_joint(ur.Joint('RFsrFR_joint', 'r_ankle',
        'RFsrFR_frame', 'fixed', None, ur.Pose((dicOffsets['FsrFROffsetX'],\
        dicOffsets['FsrFROffsetY'],dicOffsets['FsrFROffsetZ']),\
        (dicOffsets['FsrFRRotX'],dicOffsets['FsrFRRotY'],dicOffsets['FsrFRRotZ']))))

        robot.add_joint(ur.Joint('LFsrCenter_joint', 'l_ankle',
        'LFsrCenter_frame', 'fixed', None, ur.Pose((dicOffsets['FsrCenterOffsetX'],\
        dicOffsets['FsrCenterOffsetY'],dicOffsets['FsrCenterOffsetZ']),\
        (dicOffsets['FsrCenterRotX'],dicOffsets['FsrCenterRotY'],dicOffsets['FsrCenterRotZ']))))
        robot.add_joint(ur.Joint('RFsrCenter_joint', 'r_ankle',
        'RFsrCenter_frame', 'fixed', None, ur.Pose((dicOffsets['FsrCenterOffsetX'],\
        dicOffsets['FsrCenterOffsetY'],dicOffsets['FsrCenterOffsetZ']),\
        (dicOffsets['FsrCenterRotX'],dicOffsets['FsrCenterRotY'],dicOffsets['FsrCenterRotZ']))))

        robot.add_joint(ur.Joint('LFsrRCenter_joint', 'l_ankle',
        'LFsrRCenter_frame', 'fixed', None, ur.Pose((dicOffsets['FsrRCenterOffsetX'],\
        dicOffsets['FsrRCenterOffsetY'],dicOffsets['FsrRCenterOffsetZ']),\
        (dicOffsets['FsrRCenterRotX'],dicOffsets['FsrRCenterRotY'],dicOffsets['FsrRCenterRotZ']))))
        robot.add_joint(ur.Joint('RFsrRCenter_joint', 'r_ankle',
        'RFsrRCenter_frame', 'fixed', None, ur.Pose((dicOffsets['FsrRCenterOffsetX'],\
        dicOffsets['FsrRCenterOffsetY'],dicOffsets['FsrRCenterOffsetZ']),\
        (dicOffsets['FsrRCenterRotX'],dicOffsets['FsrRCenterRotY'],dicOffsets['FsrRCenterRotZ']))))

        #Touch sensors
        robot.add_link(ur.Link('HeadTouchFront_frame'))
        robot.add_link(ur.Link('HeadTouchMiddle_frame'))
        robot.add_link(ur.Link('HeadTouchRear_frame'))

        robot.add_joint(ur.Joint('HeadTouchFront_joint', 'HeadRollLink',
        'HeadTouchFront_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchFrontOffsetX'],dicOffsets['HeadTouchFrontOffsetY'],dicOffsets['HeadTouchFrontOffsetZ']),(dicOffsets['HeadTouchFrontRotX'],dicOffsets['HeadTouchFrontRotY'],dicOffsets['HeadTouchFrontRotZ']))))

        robot.add_joint(ur.Joint('HeadTouchMiddle_joint', 'HeadRollLink',
        'HeadTouchMiddle_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchMiddleOffsetX'],dicOffsets['HeadTouchMiddleOffsetY'],dicOffsets['HeadTouchMiddleOffsetZ']),(dicOffsets['HeadTouchMiddleRotX'],dicOffsets['HeadTouchMiddleRotY'],dicOffsets['HeadTouchMiddleRotZ']))))

        robot.add_joint(ur.Joint('HeadTouchRear_joint', 'HeadRollLink',
        'HeadTouchRear_frame', 'fixed', None, ur.Pose((dicOffsets['HeadTouchRearOffsetX'],dicOffsets['HeadTouchRearOffsetY'],dicOffsets['HeadTouchRearOffsetZ']),(dicOffsets['HeadTouchRearRotX'],dicOffsets['HeadTouchRearRotY'],dicOffsets['HeadTouchRearRotZ']))))

    elif name=="pepper":
        meshversion = version
       # base_footprint
        robot.add_link(ur.Link('base_footprint'))
            
        robot.add_joint(ur.Joint('base_footprint_joint', 'Tibia','base_footprint', 'fixed', None, ur.Pose((dicOffsets['BaseFootprintOffsetX'],dicOffsets['BaseFootprintOffsetY'],dicOffsets['BaseFootprintOffsetZ']),(dicOffsets['BaseFootprintRotX'],dicOffsets['BaseFootprintRotY'],dicOffsets['BaseFootprintRotZ']))))
    
        #sensor links/joints
        ## LASERS
        robot.add_link(ur.Link('ShovelLaser_frame'))
        robot.add_link(ur.Link('GroundRightLaser_frame'))
        robot.add_link(ur.Link('GroundLeftLaser_frame'))
        robot.add_link(ur.Link('SurroundingFrontLaser_frame'))
        robot.add_link(ur.Link('SurroundingRightLaser_frame'))
        robot.add_link(ur.Link('SurroundingLeftLaser_frame'))

        # ground lasers
        robot.add_joint(ur.Joint('ShovelLaser_joint', 'Tibia',
        'ShovelLaser_frame', 'fixed', None, ur.Pose((dicOffsets['ShovelLaserOffsetX'],\
        dicOffsets['ShovelLaserOffsetY'],dicOffsets['ShovelLaserOffsetZ']),\
        (dicOffsets['ShovelLaserRotX'],dicOffsets['ShovelLaserRotY'],dicOffsets['ShovelLaserRotZ']))))

        robot.add_joint(ur.Joint('GroundRightLaser_joint', 'Tibia',
        'GroundRightLaser_frame', 'fixed', None,
        ur.Pose((dicOffsets['GroundLaserOffsetX'],dicOffsets['GroundLaserOffsetY'],dicOffsets['GroundLaserOffsetZ']),\
        (dicOffsets['GroundLaserRotX'],dicOffsets['GroundLaserRotY'],dicOffsets['GroundLaserRotZ']))))

        robot.add_joint(ur.Joint('GroundLeftLaser_joint', 'Tibia', \
        'GroundLeftLaser_frame', 'fixed', None, ur.Pose((dicOffsets['GroundLaserOffsetX'],-dicOffsets['GroundLaserOffsetY'],dicOffsets['GroundLaserOffsetZ']),\
        (-dicOffsets['GroundLaserRotX'],dicOffsets['GroundLaserRotY'],-dicOffsets['GroundLaserRotZ']))))

        # surrounding lasers
        robot.add_joint(ur.Joint('SurroundingFrontLaser_joint', 'Tibia',
        'SurroundingFrontLaser_frame', 'fixed', None,
        ur.Pose((dicOffsets['SurroundingFrontLaserOffsetX'],dicOffsets['SurroundingFrontLaserOffsetY'],dicOffsets['SurroundingFrontLaserOffsetZ']),\
        (dicOffsets['SurroundingFrontLaserRotX'],dicOffsets['SurroundingFrontLaserRotY'],dicOffsets['SurroundingFrontLaserRotZ']))))

        robot.add_joint(ur.Joint('SurroundingLeftLaser_joint', 'Tibia',
        'SurroundingLeftLaser_frame', 'fixed', None,ur.Pose((dicOffsets['SurroundingLaserOffsetX'],dicOffsets['SurroundingLaserOffsetY'],dicOffsets['SurroundingLaserOffsetZ']),\
        (dicOffsets['SurroundingLaserRotX'],dicOffsets['SurroundingLaserRotY'],dicOffsets['SurroundingLaserRotZ']))))
 
        robot.add_joint(ur.Joint('SurroundingRightLaser_joint', 'Tibia', \
        'SurroundingRightLaser_frame', 'fixed', None,ur.Pose((dicOffsets['SurroundingLaserOffsetX'],-dicOffsets['SurroundingLaserOffsetY'],dicOffsets['SurroundingLaserOffsetZ']),\
        (-dicOffsets['SurroundingLaserRotX'],dicOffsets['SurroundingLaserRotY'],-dicOffsets['SurroundingLaserRotZ']))))

 
        # Cameras
        robot.add_link(ur.Link('CameraTop_frame'))
        robot.add_link(ur.Link('CameraBottom_frame'))
        robot.add_link(ur.Link('DepthCamera_frame'))


        robot.add_joint(ur.Joint('CameraTop_joint', 'Head',
        'CameraTop_frame', 'fixed', None, ur.Pose((dicOffsets['CameraTopOffsetX'],\
        dicOffsets['CameraTopOffsetY'],dicOffsets['CameraTopOffsetZ']),\
        (dicOffsets['CameraTopRotX'],dicOffsets['CameraTopRotY'],dicOffsets['CameraTopRotZ']))))
        robot.add_joint(ur.Joint('CameraBottom_joint', 'Head',
        'CameraBottom_frame', 'fixed', None,
        ur.Pose((dicOffsets['CameraBottomOffsetX'],dicOffsets['CameraBottomOffsetY'],dicOffsets['CameraBottomOffsetZ']),\
        (dicOffsets['CameraBottomRotX'],dicOffsets['CameraBottomRotY'],dicOffsets['CameraBottomRotZ']))))
        robot.add_joint(ur.Joint('DepthCamera_joint', 'Head',
        'DepthCamera_frame', 'fixed', None,
        ur.Pose((dicOffsets['DepthCameraOffsetX'],dicOffsets['DepthCameraOffsetY'],dicOffsets['DepthCameraOffsetZ']),\
        (dicOffsets['DepthCameraRotX'],dicOffsets['DepthCameraRotY'],dicOffsets['DepthCameraRotZ']))))

        # Sonars

        robot.add_link(ur.Link('SonarFront_frame'))
        robot.add_link(ur.Link('SonarBack_frame'))

        robot.add_joint(ur.Joint('SonarFront_joint', 'Tibia',
        'SonarFront_frame', 'fixed', None,
        ur.Pose((dicOffsets['SonarFrontOffsetX'],dicOffsets['SonarFrontOffsetY'],dicOffsets['SonarFrontOffsetZ']),\
        (dicOffsets['SonarFrontRotX'],dicOffsets['SonarFrontRotY'],dicOffsets['SonarFrontRotZ']))))
        robot.add_joint(ur.Joint('SonarBack_joint', 'Tibia',
        'SonarBack_frame', 'fixed', None,
        ur.Pose((dicOffsets['SonarBackOffsetX'],dicOffsets['SonarBackOffsetY'],dicOffsets['SonarBackOffsetZ']),\
        (dicOffsets['SonarBackRotX'],dicOffsets['SonarBackRotY'],dicOffsets['SonarBackRotZ']))))
        

        # Bumpers

        robot.add_link(ur.Link('BumperFL_frame'))
        robot.add_link(ur.Link('BumperFR_frame'))
        robot.add_link(ur.Link('BumperB_frame'))

        robot.add_joint(ur.Joint('BumperFR_joint', 'Tibia',
        'BumperFR_frame', 'fixed', None,
        ur.Pose((dicOffsets['FrontBumperOffsetX'],-dicOffsets['FrontBumperOffsetY'],dicOffsets['BumperOffsetZ']),\
        (dicOffsets['FrontBumperRotX'],dicOffsets['FrontBumperRotY'],-dicOffsets['FrontBumperRotZ']))))
        robot.add_joint(ur.Joint('BumperFL_joint', 'Tibia',
        'BumperFL_frame', 'fixed', None,
        ur.Pose((dicOffsets['FrontBumperOffsetX'],dicOffsets['FrontBumperOffsetY'],dicOffsets['BumperOffsetZ']),\
        (dicOffsets['FrontBumperRotX'],dicOffsets['FrontBumperRotY'],dicOffsets['FrontBumperRotZ']))))
        robot.add_joint(ur.Joint('BumperB_joint', 'Tibia',
        'BumperB_frame', 'fixed', None,
        ur.Pose((dicOffsets['BackBumperOffsetX'],dicOffsets['BackBumperOffsetY'],dicOffsets['BumperOffsetZ']),\
        (dicOffsets['BackBumperRotX'],dicOffsets['BackBumperRotY'],dicOffsets['BackBumperRotZ']))))

        # IMU
            
        robot.add_link(ur.Link('IMUAccelero_frame'))
        robot.add_link(ur.Link('IMUGyro_frame'))        

        robot.add_joint(ur.Joint('IMUAccelero_joint', 'torso',
        'IMUAccelero_frame', 'fixed', None,
        ur.Pose((dicOffsets['IMUAcceleroOffsetX'],dicOffsets['IMUAcceleroOffsetY'],dicOffsets['IMUAcceleroOffsetZ']),\
        (dicOffsets['IMUAcceleroRotX'],dicOffsets['IMUAcceleroRotY'],dicOffsets['IMUAcceleroRotZ']))))
        robot.add_joint(ur.Joint('IMUGyro_joint', 'torso',
        'IMUGyro_frame', 'fixed', None,
        ur.Pose((dicOffsets['IMUGyroOffsetX'],dicOffsets['IMUGyroOffsetY'],dicOffsets['IMUGyroOffsetZ']),\
        (dicOffsets['IMUGyroRotX'],dicOffsets['IMUGyroRotY'],dicOffsets['IMUGyroRotZ']))))


        # Touch sensors

        # Head
    
        robot.add_link(ur.Link('HeadTouchFront_frame'))
        robot.add_link(ur.Link('HeadTouchMiddle_frame'))
        robot.add_link(ur.Link('HeadTouchRear_frame'))

        robot.add_joint(ur.Joint('HeadTouchFront_joint', 'Head',
        'HeadTouchFront_frame', 'fixed', None,
        ur.Pose((dicOffsets['HeadTouchFrontOffsetX'],dicOffsets['HeadTouchFrontOffsetY'],dicOffsets['HeadTouchFrontOffsetZ']),\
        (dicOffsets['HeadTouchFrontRotX'],dicOffsets['HeadTouchFrontRotY'],dicOffsets['HeadTouchFrontRotZ']))))
        robot.add_joint(ur.Joint('HeadTouchMiddle_joint', 'Head',
        'HeadTouchMiddle_frame', 'fixed', None,
        ur.Pose((dicOffsets['HeadTouchMiddleOffsetX'],dicOffsets['HeadTouchMiddleOffsetY'],dicOffsets['HeadTouchMiddleOffsetZ']),\
        (dicOffsets['HeadTouchMiddleRotX'],dicOffsets['HeadTouchMiddleRotY'],dicOffsets['HeadTouchMiddleRotZ']))))

        robot.add_joint(ur.Joint('HeadTouchRear_joint', 'Head',
        'HeadTouchRear_frame', 'fixed', None,
        ur.Pose((dicOffsets['HeadTouchRearOffsetX'],dicOffsets['HeadTouchRearOffsetY'],dicOffsets['HeadTouchRearOffsetZ']),\
        (dicOffsets['HeadTouchRearRotX'],dicOffsets['HeadTouchRearRotY'],dicOffsets['HeadTouchRearRotZ']))))

        # Hand 
             
        robot.add_link(ur.Link('LHandTouch_frame'))
        robot.add_link(ur.Link('RHandTouch_frame')) 

        robot.add_joint(ur.Joint('LHandTouch_joint', 'l_wrist',
        'LHandTouch_frame', 'fixed', None,
        ur.Pose((dicOffsets['HandTouchBackOffsetX'],dicOffsets['HandTouchBackOffsetY'],dicOffsets['HandTouchBackOffsetZ']),\
        (dicOffsets['HandTouchBackRotX'],dicOffsets['HandTouchBackRotY'],dicOffsets['HandTouchBackRotZ']))))

        robot.add_joint(ur.Joint('RHandTouch_joint', 'r_wrist',
        'RHandTouch_frame', 'fixed', None,
        ur.Pose((dicOffsets['HandTouchBackOffsetX'],dicOffsets['HandTouchBackOffsetY'],dicOffsets['HandTouchBackOffsetZ']),\
        (dicOffsets['HandTouchBackRotX'],dicOffsets['HandTouchBackRotY'],dicOffsets['HandTouchBackRotZ']))))

##################
##### Meshes #####
##################

def includeVisualCollision(robot):
    global output
    global meshversion
    global name
    global dicLinks
    global dicVisu
    global dicOffsets
    prefix = "xacro:insert_visu_"
    doc = Document()
    root = doc.createElement("robot")
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro","http://www.ros.org/wiki/xacro")
    if name =='nao':
        node = ur.short(doc,'xacro:property','name','meshes_installed')
#        node.setAttribute('value',"$(find nao_meshes)")
        cmd = 'rospack find '+str(name)+'_meshes'
        pathdescription = subprocess.check_output(cmd, 
        stderr=subprocess.STDOUT, shell=True)
        if pathdescription =="" or not os.path.isdir(pathdescription[:-1]+'/meshes/'+meshversion):    
	    node.setAttribute('value',"false")
        else:
	    node.setAttribute('value',"true")
	
        root.appendChild(node)

    
        for link in robot.links:
	    if robot.links[link].visual != None:
	        # add xacro macro call to every link needing a visualization
	        meshname = str(dicLinks.keys()[list(dicLinks.values()).index(link)])
#	        print meshname
	        if meshname.endswith('Link'):
	            meshfile = meshname[0:-4]
	        else:
	            meshfile = meshname
	        tempVisu = ur.Visual(ur.Mesh('',(0.1,0.1,0.1)))
	        tempVisu.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/visual/' + meshfile + '.dae'
	        tempCol = ur.Collision(ur.Mesh('',(0.1,0.1,0.1)))
	        tempCol.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/collision/' +  meshfile + '.stl' 
	        robot.links[link].xacro = prefix + robot.links[link].name
	        node = ur.short(doc,'xacro:macro','name',robot.links[link].xacro)
	        node2 = ur.short(doc,'xacro:if','value',"${meshes_installed}")
	        node2.appendChild(tempVisu.to_xml(doc))
	        node2.appendChild(tempCol.to_xml(doc))

	        node.appendChild(node2)
	        try:
	            robot.links[link].visual = ur.Visual(dicVisu[meshname], ur.Material('LightGrey'),  dico.Nao_orig[meshname])
	            robot.links[link].collision = ur.Collision(dicVisu[meshname], dico.Nao_orig[meshname]) 
	            node3 = ur.short(doc,'xacro:unless','value',"${meshes_installed}")
	            node3.appendChild(robot.links[link].visual.to_xml(doc))
	            node3.appendChild(robot.links[link].collision.to_xml(doc))
	            robot.links[link].visual = None
	            robot.links[link].collision = None

	        except KeyError:
	            robot.links[link].visual = None
	            robot.links[link].collision = None
#	            print 'KeyError in dico : Nao_visu, key : ' + str(meshname)
	            node3 = ur.short(doc,'xacro:unless','value',"${meshes_installed}")
	        
	        node.appendChild(node3)

	        root.appendChild(node)
    elif name =='romeo':
        for link in robot.links:
	    if robot.links[link].visual != None:
                try:
	            meshname = str(dicLinks.keys()[list(dicLinks.values()).index(link)])
                except:
                    meshname = link
                    pass   #	        print meshname
    	        if meshname.endswith('Link'):
	            meshfile = meshname[0:-4]
	        else:
	            meshfile = meshname        # add xacro macro call to every link needing a visualization
	        print meshname

	        tempVisu = ur.Visual(ur.Mesh('',(1,1,1)))
	        tempVisu.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/visual/' + meshfile + '.dae'
	        tempCol = ur.Collision(ur.Mesh('',(1,1,1)))
	        tempCol.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/collision/' +  meshfile + '.dae' 
	        robot.links[link].xacro = prefix + robot.links[link].name
	        node = ur.short(doc,'xacro:macro','name',robot.links[link].xacro)
	        node.appendChild(tempVisu.to_xml(doc))
	        node.appendChild(tempCol.to_xml(doc))
                robot.links[link].visual = None
                robot.links[link].collision = None
	        root.appendChild(node)

    elif name == 'pepper':
         for link in robot.links:
	    if robot.links[link].visual != None:
                try:
	            meshname = str(dicLinks.keys()[list(dicLinks.values()).index(link)])
                except:
                    meshname = link
                    pass   #	        print meshname
    	        if meshname.endswith('Link'):
	            meshfile = meshname[0:-4]
	        else:
	            meshfile = meshname        # add xacro macro call to every link needing a visualization
	        print meshname

	        tempVisu = ur.Visual(ur.Mesh('',(0.1,0.1,0.1)))
	        tempVisu.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/visual/' + meshfile + '.dae'
	        tempCol = ur.Collision(ur.Mesh('',(0.1,0.1,0.1)))
	        tempCol.geometry.filename =  'package://'+name+'_description/meshes/' + str(meshversion) + '/collision/' +  meshfile + '.stl' 
	        robot.links[link].xacro = prefix + robot.links[link].name
	        node = ur.short(doc,'xacro:macro','name',robot.links[link].xacro)
	        node.appendChild(tempVisu.to_xml(doc))
	        node.appendChild(tempCol.to_xml(doc))
                robot.links[link].visual = None
                robot.links[link].collision = None
	        root.appendChild(node)
                print tempVisu.geometry.filename
    filename = output[0:output.find('.')] + 'visual_collisions.xacro'
    writeCommentedXacro(doc, filename)


#################################
######## XACRO FUNCTIONS ########
#################################

def exportXacroRobotElementList(robot,element):
    global output
    doc = Document()
    root = doc.createElement("robot")
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro","http://www.ros.org/wiki/xacro")
    print 'create element file'
    for i in robot.elements:
        try:
            if element == 'Transmission':
                if i.name.find(element) != -1:
                    root.appendChild(i.to_xml(doc))
            elif element == 'Gazebo':
                if i.reference != None:
                    root.appendChild(i.to_xml(doc))
                elif i.plugins != []:
                    root.appendChild(i.to_xml(doc))
            elif element == 'material':
                if type(i) == romeo_description.urdf.Material:
                    root.appendChild(i.to_xml(doc))
        except AttributeError:
            pass
    filename = output[0:output.find('.')]+ str(element) + '.xacro'
    print filename
    writeCommentedXacro(doc, filename)   

def exportXacroRobot(robot):
    global output
    global name
    doc = Document()
    root = doc.createElement("robot")
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro","http://www.ros.org/wiki/xacro")
    root.setAttribute("name",robot.name)
    if name == 'nao':
        root.appendChild(ur.short(doc,'xacro:include','filename',output[output.rfind('/')+1:output.find('.')]+ 'Transmission.xacro'))
        root.appendChild(ur.short(doc,'xacro:include','filename',output[output.rfind('/')+1:output.find('.')]+ 'Gazebo.xacro'))    
        root.appendChild(ur.short(doc,'xacro:include','filename',output[output.rfind('/')+1:output.find('.')]+ 'hands.xacro'))
    includeVisualCollision(robot)
    for i in dicXacro.keys():
        print "exporting " +str(dicXacro[i]) + ' xacro' 
        exportXacroRobotChain(robot,i)
        if output.find('/') != -1:
            filenamerobot = output[output.rfind('/')+1:output.find('.')]+ i + '.xacro'
        else : 
            filenamerobot = output[0:output.find('.')]+ i + str('.xacro')
        root.appendChild(ur.short(doc,'xacro:include','filename',filenamerobot))
## Transmission elements not available from Aldebaran libraries yet
#    exportXacroRobotElementList(robot,'Transmission')
## Plugin not implemented yet : neither mimic joints, camera or ros_control
#    exportXacroRobotElementList(robot,'Gazebo')
    root.appendChild(ur.short(doc,'xacro:include','filename',output[output.rfind('/')+1:output.find('.')]+ 'sensors.xacro'))
    exportSensorsXacro(robot,output[output.rfind('/')+1:output.find('.')]+ 'sensors.xacro')   
    root.appendChild(ur.short(doc,'xacro:include','filename',output[output.rfind('/')+1:output.find('.')]+ 'visual_collisions.xacro'))

    filename = output[0:output.find('.')]+ 'robot' + str('.xacro')
    print filename
    writeCommentedXacro(doc,filename)


def exportXacroRobotChain(robot, keyword):
    global output
    if name == 'nao' or name == 'pepper':
        chainRef = robot.get_chain('base_link','torso')
    elif name == 'romeo':
        chainRef = robot.get_chain('base_link','body')
    doc = Document()
    root = doc.createElement("robot")
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro","http://www.ros.org/wiki/xacro")
    chainNb =0
    try: 
        chain1 = robot.get_chain('base_link','l_'+dicXacro[keyword])
        chain2 = robot.get_chain('base_link','r_'+dicXacro[keyword])
        chainNb=2
    except KeyError:
        try:
            chain1 = robot.get_chain('base_link','L'+dicXacro[keyword])
            chain2 = robot.get_chain('base_link','R'+dicXacro[keyword])
            chainNb = 2
        except KeyError:
            try: 
                chain1 = robot.get_chain('base_link',dicXacro[keyword])
                chainNb = 1
            except KeyError:
                print 'the chain '+str(keyword) +' cannot be found'
    
    if chainNb != 0:
        duplicate = 0
        for i in range(len(chain1)):
            for j in range(len(chainRef)):
                if chain1[i] == chainRef[j]:
                    duplicate =1
            if duplicate == 0 or keyword == 'torso':
                try:
                    root.appendChild(robot.links[chain1[i]].to_xml(doc))
                except KeyError:
                    try:
                        root.appendChild(robot.joints[chain1[i]].to_xml(doc))
                    except KeyError:
                        print 'unknown element' + str(chain1[i])        
            else:
                duplicate =0
        if chainNb ==2:
            for i in range(len(chain2)):
                for j in range(len(chainRef)):
                    if chain2[i] == chainRef[j]:
                        duplicate =1
                if duplicate == 0:
                    try:
                        root.appendChild(robot.links[chain2[i]].to_xml(doc))
                    except KeyError:
                        try:
                            root.appendChild(robot.joints[chain2[i]].to_xml(doc))
                        except KeyError:
                            print 'unknown element' + str(chain2[i])        
                else:
                    duplicate =0
        filename = output[0:output.find('.')] + keyword + str('.xacro')
        print filename
        writeCommentedXacro(doc, filename)


def writeCommentedXacro(doc,filename):
    if(os.path.isdir(filename[0:filename.rfind('/')+1]) == False):
        os.makedirs(filename[0:filename.rfind('/')])

    file = open(filename,'w+') 
    file.write(doc.toprettyxml())
    file.close()
    file = open(filename,'r')
    firstline, remaining = file.readline(),file.read()
    file.close()
    file = open(filename,'w')
    file.write(firstline)
    file.write('<!-- ***************************************************************** \n'\
' ****** File automatically generated by generate_urdf.py script ****** \n'\
' ********************************************************************* -->\n')    
    file.write(remaining)
    file.close()

def exportSensorsXacro(robot,filename):
    global output
    doc = Document()
    root = doc.createElement("robot")
    doc.appendChild(root)
    root.setAttribute("xmlns:xacro","http://www.ros.org/wiki/xacro")
    root.setAttribute("name",robot.name)
 
    for link in robot.links:
        if robot.links[link].name.endswith('_frame'):
            root.appendChild(robot.links[link].to_xml(doc))  
            for joint in robot.joints:
                if robot.joints[joint].child==robot.links[link].name:
                    root.appendChild(robot.joints[joint].to_xml(doc))
  
    filename = output[0:output.find('.')] + 'sensors.xacro'
    print filename
    writeCommentedXacro(doc, filename)

##############
#### Main ####
##############

args = parser.parse_args()
if args.input is None:
    robot = URDF.from_parameter_server()
else:
    robot = URDF.load_xml_file(args.input)

if robot.name.find('V')!=-1:
    version = robot.name[robot.name.find('V'):]
else:
    version = str(args.input)[str(args.input).find("V"):str(args.input).find("V")+3] 


if version != '':
    if robot.name.lower().find('nao')!=-1:
        name = 'nao'
        try:
            import romeo_description.nao_dictionaries as dico
        except:
            print 'unable to import nao dictionaries'
            sys.exit(0)
        for element in robot.elements:
            if type(element) == romeo_description.urdf.Material:
                robot.elements.remove(element)
        dicLinks = dico.Nao_links
        dicVisu = dico.Nao_visu
        dicOffsets = dico.Nao_offsets
        dicXacro = dicoNaoXacro
    elif robot.name.lower().find('romeo')!=-1:
        name = 'romeo'
#        version = 'V1'
        try:
            import romeo_description.romeo_dictionaries as dico
        except:
            print 'unable to import romeo dictionaries'
            sys.exit(0)
        for element in robot.elements:
            if type(element) == romeo_description.urdf.Material:
                robot.elements.remove(element)
        dicLinks = dico.Romeo_links
        dicOffsets = dico.Romeo_offsets
        dicVisu = ''
        dicXacro = dicoRomeoXacro
    elif robot.name.lower().find('juliette') or robot.name.lower().find('pepper'):
        name = 'pepper'
#        version = 'V1'
        try:
            import romeo_description.pepper_dictionaries as dico
        except:
            print 'unable to import pepper dictionaries'
            sys.exit(0)
        for element in robot.elements:
            if type(element) == romeo_description.urdf.Material:
                robot.elements.remove(element)
        dicLinks = dico.Pepper_links
        dicOffsets = dico.Pepper_offsets
        dicVisu = ''
        dicXacro = dicoPepperXacro
        print "PROCESSING PEPPER ROBOT"

            
    cmd = 'rospack find '+str(name)+'_description'
    try:
        pathdescription = subprocess.check_output(cmd, 
        stderr=subprocess.STDOUT, shell=True)[:-1]
    except:
        print 'unable to find '+str(name)+'_description package'
        sys.exit(0)
    output = pathdescription+'/urdf/' + name + version +  '_generated_urdf/' + name + '.urdf'
    print'processing ' + str(name) + " robot's urdf file"
    defineMaterials(robot)
    if args.REP120=='true':
        REP120func(robot)
    if args.xacro == 'robot':
        exportXacroRobotElementList(robot,'material')
        exportXacroRobot(robot)
    elif args.xacro == 'urdf':
        print output
        robot.write_xml(output)
    else : 
        exportXacroRobotChain(robot,args.xacro)
else:
    print 'unable to retrieve robot version'
