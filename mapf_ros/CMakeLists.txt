cmake_minimum_required(VERSION 3.5)
project(mapf_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mapf_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav2_util
  tf2_ros
  nav2_costmap_2d
  builtin_interfaces
  pluginlib
  mapf_msgs
)

add_library(cbs_planner SHARED src/cbs_ros.cpp)
add_library(ecbs_planner SHARED src/ecbs_ros.cpp)
add_library(sipp_planner SHARED src/prioritized_sipp_ros.cpp)

ament_target_dependencies(cbs_planner ${dependencies})
ament_target_dependencies(ecbs_planner ${dependencies})
ament_target_dependencies(sipp_planner ${dependencies})

pluginlib_export_plugin_description_file(mapf_ros cbs_planner_plugins.xml)
pluginlib_export_plugin_description_file(mapf_ros ecbs_planner_plugins.xml)
pluginlib_export_plugin_description_file(mapf_ros sipp_planner_plugins.xml)

install(TARGETS 
  cbs_planner 
  ecbs_planner
  sipp_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES 
  cbs_planner_plugins.xml 
  ecbs_planner_plugins.xml 
  sipp_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cbs_planner)
ament_export_libraries(ecbs_planner)
ament_export_libraries(sipp_planner)
ament_export_dependencies(${dependencies})
ament_package()