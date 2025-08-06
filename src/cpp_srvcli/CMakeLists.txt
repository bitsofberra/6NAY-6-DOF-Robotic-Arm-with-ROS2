cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# .srv'den .hpp üret
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VectorDistance.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Server executable
add_executable(server src/berra_server.cpp)
ament_target_dependencies(server rclcpp)
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Geçici olarak client'i yorum satırına al (henüz yoksa)
# add_executable(client src/yilmaz_client.cpp)
# ament_target_dependencies(client rclcpp)
# rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  server
  # client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
