README to use romeo_moveit to control your Romeo
================================================

I. TO USE FROM A REMOTE COMPUTER:
=================================

I.1 Compile the package
-----------------------

Suppose that you alreadly have a ros workspace and know how to compile a ros package. In order to compile romeo_dcm package and romeo_moveit package, you need to set AL_DIR to the path to naoqiSDK-c++ on your computer. Copy romeo and romeo_dcm to your catkin workspace. Then, inside your catkin workspace, you just need to run the command *catkin_make* to compile these packages.

I.2 To run romeo_dcm and romeo_moveit:
--------------------------------------

First, set NAO_IP to your Romeo's ip.

``
roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch
``

Wait until romeo_dcm_bringup node is ready, then run:

``
roslaunch romeo_moveit_config moveit_planner.launch
``
