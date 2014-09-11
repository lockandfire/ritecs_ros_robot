cmake_minimum_required(VERSION 2.8.3)
project(ros_robot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_robot_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

add_executable(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

