cmake_minimum_required(VERSION 3.0.2)
project(multiespectral_acquire)

add_compile_options(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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
find_package(Qt5 REQUIRED COMPONENTS Widgets) # rviz plugin for cameras :)

find_package(OpenCV REQUIRED)
find_package(pylon REQUIRED)
find_package(Spinnaker REQUIRED) # Makes use of FindSpinnaker.cmake in cmake fodler

find_package(yaml-cpp REQUIRED)


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
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_msgs cv_bridge image_transport camera_info_manager actionlib actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include # Explicit path to have generated messages
  ${OpenCV_INCLUDE_DIRS}
  ${SPINNAKER_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
  # ${CMAKE_CURRENT_LIST_DIR}/acA1600_60gc_pylon_gen
  # ${CMAKE_CURRENT_LIST_DIR}/a2A2600_20gcBAS_pylon_gen
  ${CMAKE_CURRENT_LIST_DIR}/python_gui
  ${YAML_CPP_INCLUDE_DIR}
  src  # For lidar_drivers/ and utils/ headers
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

add_executable(lidar_slave src/lidar_fb_node_slave.cpp ${LIDAR_UTILS} ${MULTISPECTRAL_UTILS})


# Link libraries
foreach(target basler_master basler_slave basler_driver)
  target_link_libraries(${target} ${catkin_LIBRARIES} ${OpenCV_LIBS} pylon::pylon yaml-cpp)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

foreach(target flir_master flir_slave flir_driver)
target_link_libraries(${target} ${catkin_LIBRARIES} ${OpenCV_LIBS} pthread dl Spinnaker::Spinnaker yaml-cpp)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

foreach(target dummy_master dummy_slave dummy_driver lidar_slave)
  target_link_libraries(${target} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

install(TARGETS
  dummy_master dummy_slave dummy_driver
  flir_master flir_slave flir_driver
  basler_master basler_slave basler_driver
  lidar_slave
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