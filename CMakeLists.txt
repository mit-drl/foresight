cmake_minimum_required(VERSION 2.8.3)
project(foresight)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  PolygonArray.msg
  TreeSearchResultMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foresight
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
