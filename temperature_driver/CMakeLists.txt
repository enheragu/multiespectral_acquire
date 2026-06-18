cmake_minimum_required(VERSION 3.8)
project(temperature_driver)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemperatureHumidity.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/dht22_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
