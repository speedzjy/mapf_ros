cmake_minimum_required(VERSION 3.0.2)
project(mapf_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             message_filters
             message_generation
             nav_msgs
             roscpp
             rospy
             sensor_msgs
             std_msgs)

add_message_files(FILES Goal.msg SinglePlan.msg GlobalPlan.msg)

generate_messages(DEPENDENCIES std_msgs nav_msgs mapf_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_generation nav_msgs
               sensor_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
