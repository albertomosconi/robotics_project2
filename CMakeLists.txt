cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(broadcast_odom_tf src/broadcast_odom_tf.cpp)
target_link_libraries(broadcast_odom_tf ${catkin_LIBRARIES})