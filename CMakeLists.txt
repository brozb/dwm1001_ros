cmake_minimum_required(VERSION 3.0.2)
project(dwm1001_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_numpy
  rospy
  std_msgs
  tf2
  tf2_py
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  Anchor.msg
  TagLocation.msg
  UWBMeas.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs  std_msgs
)

catkin_package(
)

catkin_install_python(PROGRAMS
    scripts/uwb_tag.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
