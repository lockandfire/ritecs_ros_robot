cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_ctrlpub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(ros_robot_ctrlpub src/publisher.cpp)
target_link_libraries(ros_robot_ctrlpub
   ${catkin_LIBRARIES}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
