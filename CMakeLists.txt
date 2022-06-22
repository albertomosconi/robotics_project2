cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  grid_map_msgs
  grid_map_ros
  grid_map_cv
  cv_bridge
  tf2
  tf2_ros
)

## Generate services in the 'srv' folder
add_service_files(FILES save_map_trajectory.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs grid_map_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(broadcast_odom_tf src/broadcast_odom_tf.cpp)
target_link_libraries(broadcast_odom_tf ${catkin_LIBRARIES})

add_executable(save_map_trajectory src/save_map_trajectory.cpp)
target_link_libraries(save_map_trajectory ${catkin_LIBRARIES})
add_dependencies(save_map_trajectory ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
