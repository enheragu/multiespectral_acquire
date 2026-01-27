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

# Ensure the Python package is processed via setup.py during the catkin build
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge actionlib actionlib_msgs message_runtime multiespectral_acquire
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

install(DIRECTORY multiespectral_acquire_gui/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Ensure individual python module files are present in the installed package
# so submodules like multiespectral_acquire_gui.multiespectral_ros_ac can be
# imported directly from dist-packages (some catkin setups only deploy a
# package __init__.py that links to source; copying the .py files here makes
# imports reliable in all cases).
install(FILES
  multiespectral_acquire_gui/FreqCounter.py
  multiespectral_acquire_gui/multiespectral_ros_ac.py
  multiespectral_acquire_gui/multiespectral_dummy_ac.py
  multiespectral_acquire_gui/multiespectral_control.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/multiespectral_acquire_gui
)

# Install only the entry-point script as an executable. Package modules
# are installed via the directory install above into
# ${CATKIN_PACKAGE_PYTHON_DESTINATION} so they are available as
# multiespectral_acquire_gui.* submodules and won't be created as
# top-level modules under devel/lib which cause import shadowing.
catkin_install_python(PROGRAMS
  # multiespectral_acquire_gui/FreqCounter.py  
  multiespectral_acquire_gui/multiespectral_control.py
  # multiespectral_acquire_gui/multiespectral_ros_ac.py
  # multiespectral_acquire_gui/multiespectral_dummy_ac.py
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
