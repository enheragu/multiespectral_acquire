cmake_minimum_required(VERSION 3.0.2)
project(multiespectral_acquire_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  actionlib
  actionlib_msgs
  message_generation
  multiespectral_acquire
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge actionlib actionlib_msgs message_runtime multiespectral_acquire
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Instala scripts python, html, static, etc.
catkin_install_python(PROGRAMS
  multiespectral_acquire_gui/multiespectral_control.py
  multiespectral_acquire_gui/multiespectral_ros_ac.py
  multiespectral_acquire_gui/multiespectral_dummy_ac.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY multiespectral_acquire_gui/templates/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/templates
)
install(DIRECTORY multiespectral_acquire_gui/static/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/static
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
