cmake_minimum_required(VERSION 2.8.3)
project(tutorial2_support)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

#############
## Install ##
#############

install(DIRECTORY config meshes urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
