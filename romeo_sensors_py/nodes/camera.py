#!/usr/bin/env python
import rospy
from romeo_sensors.romeo_camera import RomeoCam

if __name__ == "__main__":
  romeocam = RomeoCam('romeo_camera')
  romeocam.start()
  rospy.spin()
