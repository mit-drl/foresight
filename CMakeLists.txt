cmake_minimum_required(VERSION 2.8.3)
project(foresight)

#add dynamic reconfigure api
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
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

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
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