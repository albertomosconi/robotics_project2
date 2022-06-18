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

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)
  
add_executable(merge_scans src/merge_scans.cpp)
target_link_libraries(merge_scans ${catkin_LIBRARIES})

add_executable(odom_tf src/odom_tf.cpp)
target_link_libraries(odom_tf ${catkin_LIBRARIES})