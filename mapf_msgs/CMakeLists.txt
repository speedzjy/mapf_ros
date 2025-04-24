cmake_minimum_required(VERSION 3.8)
project(mapf_msgs)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Goal.msg"
  "msg/SinglePlan.msg"
  "msg/GlobalPlan.msg"
  DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs
)

ament_package()

