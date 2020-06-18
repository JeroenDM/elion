cmake_minimum_required(VERSION 2.8.3)
project(elion_moveit_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  pluginlib
  roscpp
  elion_planner
)

# TODO I don't get it working to get this dependency
# through depending on elion_planner
find_package(ompl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS moveit_core moveit_msgs pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS} # TODO see line 16
)

add_library(
  ${PROJECT_NAME}
  src/elion_planner_manager.cpp
  src/elion_planning_context.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES} # TODO see line 16
)