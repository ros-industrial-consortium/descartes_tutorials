cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial1
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
