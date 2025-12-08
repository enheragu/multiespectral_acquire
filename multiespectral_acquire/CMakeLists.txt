cmake_minimum_required(VERSION 3.8)
project(multiespectral_acquire)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(pylon REQUIRED)
find_package(Spinnaker REQUIRED) # Makes use of FindSpinnaker.cmake in cmake fodler
find_package(yaml-cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiespectralAcquisition.action"
  "srv/ImageRequest.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Executables 
add_executable(dummy_master src/multiespectral_fb_node_master.cpp src/camera_drivers/dummy_adapter.cpp src/camera_drivers/camera_adapter.cpp )
add_executable(dummy_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/dummy_adapter.cpp src/camera_drivers/camera_adapter.cpp )
add_executable(dummy_driver src/multiespectral_fb_node_driver.cpp src/camera_drivers/dummy_adapter.cpp src/camera_drivers/camera_adapter.cpp )

add_executable(flir_master src/multiespectral_fb_node_master.cpp src/camera_drivers/flir_adapter.cpp src/camera_drivers/camera_adapter.cpp )
add_executable(flir_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/flir_adapter.cpp src/camera_drivers/camera_adapter.cpp )

add_executable(basler_master src/multiespectral_fb_node_master.cpp src/camera_drivers/basler_adapter.cpp src/camera_drivers/camera_adapter.cpp )
add_executable(basler_slave src/multiespectral_fb_node_slave.cpp src/camera_drivers/basler_adapter.cpp src/camera_drivers/camera_adapter.cpp )

## ROS stuff dependencies
ament_target_dependencies(dummy_master rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)
ament_target_dependencies(dummy_slave rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)
ament_target_dependencies(dummy_driver rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)

ament_target_dependencies(flir_master rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)
ament_target_dependencies(flir_slave rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)

ament_target_dependencies(basler_master rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)
ament_target_dependencies(basler_slave rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs rclcpp_action)

## Link libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dummy_master yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target})
target_link_libraries(dummy_slave yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target})
target_link_libraries(dummy_driver yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target})

target_link_libraries(flir_master yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target} pthread dl Spinnaker::Spinnaker)
target_link_libraries(flir_slave yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target} pthread dl Spinnaker::Spinnaker)

target_link_libraries(basler_master yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target} pthread dl pylon::pylon)
target_link_libraries(basler_slave yaml-cpp ${OpenCV_LIBS} ${cpp_typesupport_target} pthread dl pylon::pylon)


## Install targets, launch files and configuration
install(TARGETS
    dummy_master
    dummy_slave
    dummy_driver
    flir_master
    flir_slave
    basler_master
    basler_slave
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY conf/
  DESTINATION share/${PROJECT_NAME}/conf
)

ament_package()