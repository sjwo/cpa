cmake_minimum_required(VERSION 2.8.3)
project(cpa)

find_package(catkin REQUIRED COMPONENTS
  marine_msgs
  nav_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES posmv
#  CATKIN_DEPENDS geometry_msgs marine_msgs rospy
#  DEPENDS system_lib
)

