cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_odompub)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  gazebo_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_robot_odompub_node src/ros_robot_odompub.cpp)
target_link_libraries(ros_robot_odompub_node
   ${catkin_LIBRARIES}
)
