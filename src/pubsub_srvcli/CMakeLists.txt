cmake_minimum_required(VERSION 3.8)
project(pubsub_srvcli)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(nlohmann_json REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/VectorDistance.srv"
  "srv/AddThreeInts.srv"
  DEPENDENCIES std_msgs
)

# Include generated headers and local headers
include_directories(
  include
)

# Executables
## Custom subscriber
add_executable(custom_subscriber src/subscriber_custom_msg.cpp)
ament_target_dependencies(custom_subscriber
  rclcpp
  std_msgs
  rosidl_default_runtime
)
rosidl_target_interfaces(custom_subscriber
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Custom publisher
add_executable(custom_publisher src/publisher_custom_msg.cpp)
ament_target_dependencies(custom_publisher
  rclcpp
  std_msgs
  rosidl_default_runtime
)
rosidl_target_interfaces(custom_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Standard subscriber
add_executable(subscriber src/subscriber_std_msg.cpp)
ament_target_dependencies(subscriber
  rclcpp
  std_msgs
  rosidl_default_runtime
)
rosidl_target_interfaces(subscriber
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Standard publisher
add_executable(publisher src/publisher_std_msg.cpp)
ament_target_dependencies(publisher
  rclcpp
  std_msgs
  rosidl_default_runtime
)
rosidl_target_interfaces(publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Custom service server
add_executable(custom_server src/server_custom_msg.cpp)
ament_target_dependencies(custom_server
  rclcpp
  rosidl_default_runtime
)
target_link_libraries(custom_server
  nlohmann_json::nlohmann_json
)
rosidl_target_interfaces(custom_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Custom service client
add_executable(custom_client src/client_custom_msg.cpp)
ament_target_dependencies(custom_client
  rclcpp
  rosidl_default_runtime
)
target_link_libraries(custom_client
  nlohmann_json::nlohmann_json
)
rosidl_target_interfaces(custom_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Standard service server
add_executable(server src/server_std_msg.cpp)
ament_target_dependencies(server
  rclcpp
  rosidl_default_runtime
)
rosidl_target_interfaces(server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

## Standard service client
add_executable(client src/client_std_msg.cpp)
ament_target_dependencies(client
  rclcpp
  rosidl_default_runtime
)
rosidl_target_interfaces(client
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets
install(
  TARGETS
    custom_subscriber
    custom_publisher
    subscriber
    publisher
    custom_server
    custom_client
    server
    client
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()