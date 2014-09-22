cmake_minimum_required(VERSION 2.8.3)
project(romeo_sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nao_sensors
)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# install camera scripts
install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)

