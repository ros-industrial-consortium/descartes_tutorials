cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslib
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
  tf
  tf_conversions
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Tutorial 1
add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)

target_link_libraries(${PROJECT_NAME}_tutorial1
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")

# Tutorial 2 - Puzzle Demo
add_executable(${PROJECT_NAME}_tutorial2 src/tutorial2.cpp)

target_link_libraries(${PROJECT_NAME}_tutorial2
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_tutorial2
  PROPERTIES OUTPUT_NAME tutorial2
  PREFIX "")



#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1 ${PROJECT_NAME}_tutorial2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
