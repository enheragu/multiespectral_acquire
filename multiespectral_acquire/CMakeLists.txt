cmake_minimum_required(VERSION 3.0.2)
project(multiespectral_acquire)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES MultiespectralAcquisition.action
)
add_service_files(
  DIRECTORY srv
  FILES ImageRequest.srv
)
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport camera_info_manager actionlib actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/multiespectral_acquire
)

# Utils and core sources
set(MULTISPECTRAL_UTILS
  src/utils/image_metadata.cpp
  src/utils/logging_utils.h
)
set(CAMERA_CORE
  src/camera_drivers/camera_adapter.cpp
  src/camera_adapter_ros.cpp
)

# Executables (dummy, flir, basler)
add_executable(dummy_master src/multiespectral_fb_node_master.cpp src/camera_drivers/dummy_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(dummy_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/dummy_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(dummy_driver src/multiespectral_fb_node_driver.cpp src/camera_drivers/dummy_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})

add_executable(flir_master src/multiespectral_fb_node_master.cpp src/camera_drivers/flir_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(flir_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/flir_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(flir_driver src/multiespectral_fb_node_driver.cpp src/camera_drivers/flir_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})

add_executable(basler_master src/multiespectral_fb_node_master.cpp src/camera_drivers/basler_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(basler_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/basler_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(basler_driver src/multiespectral_fb_node_driver.cpp src/camera_drivers/basler_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})

# Link libraries
foreach(target dummy_master dummy_slave dummy_driver flir_master flir_slave flir_driver basler_master basler_slave basler_driver)
  target_link_libraries(${target} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

install(TARGETS
  dummy_master dummy_slave dummy_driver
  flir_master flir_slave flir_driver
  basler_master basler_slave basler_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY conf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf
)