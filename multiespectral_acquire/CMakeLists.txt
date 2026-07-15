cmake_minimum_required(VERSION 3.8)
project(multiespectral_acquire)

add_compile_options(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pylon REQUIRED)
find_package(Spinnaker REQUIRED)
find_package(yaml-cpp REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImageMetadata.msg"
  "msg/ImageWithMetadata.msg"
  "msg/TriggerStamp.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Get generated C++ typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Shared include dirs
include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  src
  ${OpenCV_INCLUDE_DIRS}
  ${SPINNAKER_INCLUDE_DIRS}
)

# Shared source sets
set(MULTISPECTRAL_UTILS src/core/utils/image_metadata.cpp)
set(CAMERA_CORE src/core/camera_drivers/camera_adapter.cpp)

# ---- Camera handler nodes (one binary per camera driver) ----
add_executable(basler_camera_handler
  src/camera_handler_node.cpp
  src/core/camera_drivers/basler_adapter.cpp
  ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
ament_target_dependencies(basler_camera_handler
  rclcpp sensor_msgs std_msgs cv_bridge image_transport camera_info_manager)
target_link_libraries(basler_camera_handler
  "${cpp_typesupport_target}" ${OpenCV_LIBS} pylon::pylon yaml-cpp)

add_executable(flir_camera_handler
  src/camera_handler_node.cpp
  src/core/camera_drivers/flir_adapter.cpp
  ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
ament_target_dependencies(flir_camera_handler
  rclcpp sensor_msgs std_msgs cv_bridge image_transport camera_info_manager)
target_link_libraries(flir_camera_handler
  "${cpp_typesupport_target}" ${OpenCV_LIBS} pthread dl Spinnaker::Spinnaker yaml-cpp)

add_executable(dummy_camera_handler
  src/camera_handler_node.cpp
  src/core/camera_drivers/dummy_adapter.cpp
  ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
ament_target_dependencies(dummy_camera_handler
  rclcpp sensor_msgs std_msgs cv_bridge image_transport camera_info_manager)
target_link_libraries(dummy_camera_handler
  "${cpp_typesupport_target}" ${OpenCV_LIBS} yaml-cpp)

# ---- Crop nodes ----
add_executable(pointcloud_crop_node src/pointcloud_crop_node.cpp)
ament_target_dependencies(pointcloud_crop_node rclcpp sensor_msgs)

add_executable(image_crop_node src/image_crop_node.cpp)
ament_target_dependencies(image_crop_node rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(image_crop_node ${OpenCV_LIBS})

# ---- Ouster sync node ----
add_executable(ouster_sync_node src/ouster_sync_node.cpp)
ament_target_dependencies(ouster_sync_node rclcpp sensor_msgs std_msgs)
target_link_libraries(ouster_sync_node "${cpp_typesupport_target}")

# ---- Install ----
install(TARGETS
  basler_camera_handler flir_camera_handler dummy_camera_handler
  pointcloud_crop_node image_crop_node
  ouster_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/buffer_compositor_node.py
  scripts/buffer_handler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY conf/
  DESTINATION share/${PROJECT_NAME}/conf
)

ament_package()
