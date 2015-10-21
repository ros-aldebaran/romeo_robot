README romeo_sensors_py
=======================

This ROS package enables communication with cameras via NAOqi. 
It can be used either from a computer that connects to the robot via network or locally on the robot itself.

I. TO USE FROM A REMOTE COMPUTER
================================

I.2 Run romeo_sensors_py
------------------------

romeo sensors needs NAO_IP in order to get images from Romeo's cameras. So before running nao_camera, please set NAO_IP to your robot's ip.

If your catkin workspace is correctly sourced, you can run the following command without error:

      roslaunch romeo_sensors_py cameras.launch

Now you can use image_view or rviz to visualize the image(s) that publishes by romeo_sensors_py. If you run rostopic list, you should see the following topics in the returned list:

      /camera_depth/camera_info
      /camera_depth/image_raw
      /camera_depth/image_color

II. TO USE ON YOUR ROMEO
=========================

II.1 Prepare romeo_sensors_py
-----------------------------

In order to use it on your robot, you need to compile the package in a naoqi environment. Compiling any ROS package in naoqi is quite simple, as explained [here!](http://wiki.ros.org/nao/Installation/compileFromVirtualNao):

II.2 Run romeo_sensors_py
=========================

Once you have your romeo_sensors_py compiled and installed on your robot, you can simply run 

roslaunch romeo_sensors_py cameras.launch

Now you can use image_view or rviz to visualize the image(s) that publishes by romeo_sensors_py. If you run rostopic list, you should see the following topics in the returned list:

      /camera_depth/camera_info
      /camera_depth/image_raw
      /camera_depth/image_color

Now you can use image_view to visualize the image(s) that publishes by romeo_sensors_py. From remote computer, you can also run image_view to view images from romeo_sensors_py, you just need to export ROS_MASTER_URI, ROS_IP, and ROS_HOSTNAME with the right information.

TODO: Currently this node assumes that depth camera on romeo is from source 2, as declared in the launch file. To be tested on the complete romeo where the 4 rgb cameras and depth camera are all installed. Things should be done:
   - Handler to control the 4 rgb cameras and 1 depth camera on complete Romeo
   - Test on Romeo with tabletop
   - Attach camera node to a given link/frame (for stereo camera and depth camera)
   - merge with nao_sensors
