cmake_minimum_required(VERSION 3.5)
project(mapf_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mapf_msgs REQUIRED)
find_package(mapf_ros REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  nav2_util
  tf2_ros
  nav2_costmap_2d
  builtin_interfaces
  pluginlib
  mapf_msgs
  mapf_ros
)

# mapf_base
add_library(mapf_base SHARED src/mapf_base.cpp)
ament_target_dependencies(mapf_base ${dependencies})

# mapf_base_node
add_executable(mapf_base_node src/mapf_base_node.cpp)
target_link_libraries(mapf_base_node mapf_base)
ament_target_dependencies(mapf_base_node ${dependencies})

install(
  TARGETS mapf_base_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# sub goal of each agent | pub goal in mapf form
add_executable(goal_transformer example/goal_transformer.cpp)
ament_target_dependencies(goal_transformer ${dependencies})

install(
  TARGETS goal_transformer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# sub plan of mapf result | use move_base client to execute plan
add_executable(plan_executor example/plan_executor.cpp)
ament_target_dependencies(plan_executor ${dependencies})

install(
  TARGETS plan_executor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
