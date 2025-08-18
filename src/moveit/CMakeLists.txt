cmake_minimum_required(VERSION 3.10)
project(moveit LANGUAGES CXX C)

# Standartlar
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Paketler
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---- SRV arayüzü ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VectorDistance.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- Executables ----
add_executable(custom_server src/moveit_server.cpp)
ament_target_dependencies(custom_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(custom_server ${cpp_typesupport_target})
add_dependencies(custom_server ${PROJECT_NAME}__rosidl_generator_cpp)


add_executable(planning_scene_relay src/planning_scene_relay.cpp)
ament_target_dependencies(planning_scene_relay
  rclcpp
  moveit_msgs
)
install(TARGETS planning_scene_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME})



add_executable(custom_client src/moveit_client.cpp)
ament_target_dependencies(custom_client
  rclcpp
  nlohmann_json
)
target_link_libraries(custom_client
  ${cpp_typesupport_target}
  nlohmann_json::nlohmann_json
)
add_dependencies(custom_client ${PROJECT_NAME}__rosidl_generator_cpp)

# ---- Kurulum ----
install(TARGETS
  custom_server
  custom_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY srv/ DESTINATION share/${PROJECT_NAME}/srv)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# (Varsa) URDF klasörün de
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()



