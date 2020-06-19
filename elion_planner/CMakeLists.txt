cmake_minimum_required(VERSION 2.8.3)
project(elion_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  moveit_msgs
  roscpp
)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core moveit_msgs
  # DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/bounds.cpp
  src/conversions.cpp
  src/msg_parsing.cpp
  src/constraint.cpp
  src/planner.cpp
  src/collision_checking.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/
  DESTINATION
    ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)

  find_package(moveit_ros_planning REQUIRED)
  find_package(rostest REQUIRED)

  # It seems I don't need the line below, I don't know why...
  # include_directories(${moveit_ros_planning_INCLUDE_DIRS})

  catkin_add_gtest(test_elion_conversions test/test_conversions.cpp)

  target_link_libraries(test_elion_conversions
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # add_rostest_gtest(${PROJECT_NAME}_test_constraints_kuka_kr5
  #   test/test_constraints_kuka_kr5.rostest
  #   test/test_constraints.cpp
  # )

  # target_link_libraries(${PROJECT_NAME}_test_constraints_kuka_kr5
  #   gtest
  #   ${moveit_ros_planning_LIBRARIES}
  #   ${catkin_LIBRARIES}
  #   ${PROJECT_NAME}
  # )

  add_rostest_gtest(${PROJECT_NAME}_test_constraints_panda
    test/test_constraints_panda.rostest
    test/test_constraints.cpp
  )

  target_link_libraries(${PROJECT_NAME}_test_constraints_panda
    gtest
    ${moveit_ros_planning_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )

endif()