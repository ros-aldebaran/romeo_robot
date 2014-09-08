#!/usr/bin/env python

#
# ROS node to provide access to the camera by wrapping NaoQI access (may not
# be the most efficient way...)
#
# Copyright 2012 Daniel Maier, University of Freiburg
# Copyright 2014 Aldebaran Robotics
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from collections import defaultdict
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nao_driver.nao_driver_naoqi import NaoNode
import camera_info_manager
from sensor_msgs.msg._CameraInfo import CameraInfo

#from dynamic_reconfigure.server import Server
#from nao_camera.cfg import NaoCameraConfig

from naoqi import ALProxy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# import resolutions
from nao_camera.vision_definitions import k960p, k4VGA, kVGA, kQVGA, kQQVGA
# import color spaces
from nao_camera.vision_definitions import kYUV422ColorSpace, kYUVColorSpace, \
                    kRGBColorSpace, kBGRColorSpace, kDepthColorSpace
# import extra parameters
from nao_camera.vision_definitions import kCameraSelectID, kCameraAutoExpositionID, kCameraAecAlgorithmID, \
                  kCameraContrastID, kCameraSaturationID, kCameraHueID, kCameraSharpnessID, kCameraAutoWhiteBalanceID, \
                  kCameraExposureID, kCameraGainID, kCameraBrightnessID, kCameraWhiteBalanceID

# those should appear in vision_definitions.py at some point
kTopCamera = 0
kBottomCamera = 1
kDepthCamera = 2

class NaoCam (NaoNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_camera')

        self.camProxy = self.getProxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)
        self.nameId = None
        self.camera_infos = {}
        def returnNone():
            return None

        self.bridge = CvBridge()
        # ROS publishers
        self.pub_img_ = rospy.Publisher('~image_raw', Image)
        self.pub_cimg_ = rospy.Publisher('~image_color', Image)
        self.pub_info_ = rospy.Publisher('~camera_info', CameraInfo)

        # initialize the parameter of the depth camera
	self.setup_cam()

    def setup_cam( self):
        # unsubscribe for all zombie subscribers
        self.camProxy.unsubscribeAllInstances("rospy_gvm")
        # subscribe
	resolution = 1 #320x240
	framerate  = 15
	color_space= 17 # mono16
        self.nameId = self.camProxy.subscribe("rospy_gvm", resolution, color_space, framerate)
        rospy.loginfo('Using camera: depth camera. Subscriber name is %s .' % (self.nameId))

	self.frame_id = rospy.get_param("~frame_id", "/CameraDepth_frame");

        self.camProxy.setResolution(self.nameId, resolution)
        self.camProxy.setFrameRate(self.nameId, framerate)
        self.camProxy.setColorSpace(self.nameId, color_space)#depth
        return

    def main_loop(self):
        img = Image()
        cimg = Image()
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.pub_img_.get_num_connections() == 0:
                r.sleep()
                continue

            image = self.camProxy.getImageRemote(self.nameId)
            if image is None:
                continue
            # Deal with the image
            '''
            #Images received from NAO have
            if self.config['use_ros_time']:
                img.header.stamp = rospy.Time.now()
            else:
                img.header.stamp = rospy.Time(image[4] + image[5]*1e-6)
            '''
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = self.frame_id
            img.height = image[1]
            img.width = image[0]
            nbLayers = image[2]
            if image[3] == kYUVColorSpace:
                encoding = "mono8"
            elif image[3] == kRGBColorSpace:
                encoding = "rgb8"
            elif image[3] == kBGRColorSpace:
                encoding = "bgr8"
            elif image[3] == kYUV422ColorSpace:
                encoding = "yuv422" # this works only in ROS groovy and later
            elif image[3] == kDepthColorSpace:
                encoding = "mono16"
            else:
                rospy.logerr("Received unknown encoding: {0}".format(image[3]))
            img.encoding = encoding
            img.step = img.width * nbLayers
            img.data = image[6]

            self.pub_img_.publish(img)
            
            # deal with the camera info
            infomsg = CameraInfo()
            infomsg.header = img.header
            # yes, this is only for an XTion / Kinect but that's the only thing supported by NAO
            ratio_x = float(640)/float(img.width)
            ratio_y = float(480)/float(img.height)
            infomsg.width = img.width
            infomsg.height = img.height
            # [ 525., 0., 3.1950000000000000e+02, 0., 525., 2.3950000000000000e+02, 0., 0., 1. ]
            infomsg.K = [ 525, 0, 3.1950000000000000e+02,
                         0, 525, 2.3950000000000000e+02,
                         0, 0, 1 ]
            infomsg.P = [ 525, 0, 3.1950000000000000e+02, 0,
                         0, 525, 2.3950000000000000e+02, 0,
                         0, 0, 1, 0 ]

            for i in range(3):
                infomsg.K[i] = infomsg.K[i] / ratio_x
                infomsg.K[3+i] = infomsg.K[3+i] / ratio_y
                infomsg.P[i] = infomsg.P[i] / ratio_x
                infomsg.P[4+i] = infomsg.P[4+i] / ratio_y

            infomsg.D = []
            infomsg.binning_x = 0
            infomsg.binning_y = 0
            infomsg.distortion_model = ""
            self.pub_info_.publish(infomsg)

	    #Currently we only get depth image from the 3D camera of NAO, so we make up a fake color image (a black image)
            #and publish it under image_color topic.
            #This should be update when the color image from 3D camera becomes available.
            colorimg = np.zeros((img.height,img.width,3), np.uint8)
            try:
              cimg = self.bridge.cv2_to_imgmsg(colorimg, "bgr8")
              cimg.header.stamp = img.header.stamp
              cimg.header.frame_id = img.header.frame_id
              self.pub_cimg_.publish(cimg)
            except CvBridgeError, e:
              print e

            r.sleep()


        self.camProxy.unsubscribe(self.nameId)

if __name__ == "__main__":
    try:
        naocam = NaoCam()
        naocam.main_loop()
    except RuntimeError as e:
        rospy.logerr('Something went wrong: %s' % str(e) )
    rospy.loginfo('Camera stopped')
