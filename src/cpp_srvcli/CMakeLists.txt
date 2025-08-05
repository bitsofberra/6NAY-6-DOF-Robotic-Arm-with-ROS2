cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(server src/service.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)     

add_executable(client src/client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      # CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()