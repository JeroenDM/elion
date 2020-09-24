cmake_minimum_required(VERSION 2.8.3)
project(elion_examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(jsoncpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  moveit_planners_ompl
  elion_planner
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elion_examples
#  CATKIN_DEPENDS moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_util
  src/util.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_util
  ${catkin_LIBRARIES}
  jsoncpp_lib
)

add_executable(${PROJECT_NAME}_kuka_example_with_collision  src/kuka_example_with_collision.cpp)

target_link_libraries(${PROJECT_NAME}_kuka_example_with_collision
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_util
)

add_executable(elion_run_example  src/run_example.cpp)

target_link_libraries(elion_run_example
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_util
  jsoncpp_lib
)

add_executable(elion_visualize_ompl_constraints src/visualize_ompl_constraints.cpp)
target_link_libraries(elion_visualize_ompl_constraints ${catkin_LIBRARIES})
set_target_properties(elion_visualize_ompl_constraints PROPERTIES OUTPUT_NAME "visualize_ompl_constraints")
