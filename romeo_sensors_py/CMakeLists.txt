cmake_minimum_required(VERSION 2.8.3)
project(romeo_sensors_py)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS nodes/camera.py nodes/camera_depth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
# Instructions to install launch files
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)

