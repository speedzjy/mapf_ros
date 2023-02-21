cmake_minimum_required(VERSION 3.0.2)
project(mapf_base)

find_package(
  catkin REQUIRED
  COMPONENTS costmap_2d
             geometry_msgs
             mapf_msgs
             nav_msgs
             roscpp
             rospy
             sensor_msgs
             std_msgs
             tf2_geometry_msgs
             tf2_ros
             pluginlib
             mapf_ros)

catkin_package(
  INCLUDE_DIRS
  include
  # LIBRARIES mapf
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  mapf_msgs
  costmap_2d
  pluginlib
  mapf_ros
  # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

# mapf_base
add_library(mapf_base src/mapf_base.cpp)
target_link_libraries(mapf_base ${catkin_LIBRARIES})
add_dependencies(mapf_base ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# mapf_base_node
add_executable(mapf_base_node src/mapf_base_node.cpp)
add_dependencies(mapf_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapf_base_node mapf_base)
set_target_properties(mapf_base_node PROPERTIES OUTPUT_NAME mapf_base)

# sub goal of each agent | pub goal in mapf form
add_executable(goal_transformer example/goal_transformer.cpp)
add_dependencies(goal_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_transformer ${catkin_LIBRARIES})

# sub plan of mapf result | use move_base client to execute plan
add_executable(plan_executor example/plan_executor.cpp)
add_dependencies(plan_executor ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_executor ${catkin_LIBRARIES})

install(TARGETS mapf_base_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS mapf_base DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")
