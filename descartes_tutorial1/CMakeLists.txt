cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorial1)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(tutorial1 src/tutorial1.cpp)

 target_link_libraries(tutorial1
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
