cmake_minimum_required(VERSION 3.0.2)
project(mapf_ros)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
             nav_msgs
             geometry_msgs
             sensor_msgs
             mapf_msgs
             costmap_2d
             tf2_geometry_msgs
             tf2_ros
             move_base_msgs
             actionlib
             roslib
             pluginlib)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  cbs_planner_ros
  ecbs_planner_ros
  sipp_planner_ros
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  mapf_msgs
  costmap_2d
  tf2_geometry_msgs
  tf2_ros
  move_base_msgs
  actionlib
  roslib
  pluginlib)

include_directories(include ${catkin_INCLUDE_DIRS})

# cbs
add_library(cbs_planner_ros src/cbs_ros.cpp)
add_dependencies(cbs_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbs_planner_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# ecbs
add_library(ecbs_planner_ros src/ecbs_ros.cpp)
add_dependencies(ecbs_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ecbs_planner_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# sipp
add_library(sipp_planner_ros src/prioritized_sipp_ros.cpp)
add_dependencies(sipp_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(sipp_planner_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS cbs_planner_ros ecbs_planner_ros sipp_planner_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES cbs_plugins.xml ecbs_plugins.xml sipp_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")
