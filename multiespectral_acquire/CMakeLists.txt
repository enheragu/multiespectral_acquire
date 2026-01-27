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
  message_generation
)

find_package(Qt5 REQUIRED COMPONENTS Widgets) # rviz plugin for cameras :)
find_package(OpenCV REQUIRED)
find_package(pylon REQUIRED)
find_package(Spinnaker REQUIRED) # Makes use of FindSpinnaker.cmake in cmake folder
find_package(yaml-cpp REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES 
    ImageMetadata.msg
    ImageWithMetadata.msg
    TriggerStamp.msg
)
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport camera_info_manager message_runtime
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
  src  # For core/ and ros_utils/ headers
)

# Utils and core sources
set(MULTISPECTRAL_UTILS
  src/core/utils/image_metadata.cpp
  src/core/utils/logging_utils.h
)
set(CAMERA_CORE
  src/core/camera_drivers/camera_adapter.cpp
)

# New simplified camera handler nodes
add_executable(basler_camera_handler src/camera_handler_node.cpp src/core/camera_drivers/basler_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(flir_camera_handler src/camera_handler_node.cpp src/core/camera_drivers/flir_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(dummy_camera_handler src/camera_handler_node.cpp src/core/camera_drivers/dummy_adapter.cpp ${CAMERA_CORE} ${MULTISPECTRAL_UTILS})
add_executable(pointcloud_crop_node src/pointcloud_crop_node.cpp ${MULTISPECTRAL_UTILS})
add_executable(image_crop_node src/image_crop_node.cpp ${MULTISPECTRAL_UTILS})

# Link libraries for new nodes
target_link_libraries(basler_camera_handler ${catkin_LIBRARIES} ${OpenCV_LIBS} pylon::pylon yaml-cpp)
add_dependencies(basler_camera_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(flir_camera_handler ${catkin_LIBRARIES} ${OpenCV_LIBS} pthread dl Spinnaker::Spinnaker yaml-cpp)
add_dependencies(flir_camera_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dummy_camera_handler ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(dummy_camera_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pointcloud_crop_node ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(pointcloud_crop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_crop_node ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(image_crop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/buffer_handler_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  basler_camera_handler flir_camera_handler dummy_camera_handler pointcloud_crop_node image_crop_node
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