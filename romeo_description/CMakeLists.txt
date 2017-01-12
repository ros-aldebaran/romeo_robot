cmake_minimum_required(VERSION 2.8.3)
project(romeo_description)

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED)

# Nothing that can be imported by other modules
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
# Instructions to install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config )
# Instructions to install urdf files
install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
# Instructions to install meshes files
install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

