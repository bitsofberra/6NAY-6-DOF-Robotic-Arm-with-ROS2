cmake_minimum_required(VERSION 3.8)
project(pubsub_srvcli)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)

# ---- Interfaces (msgs/srvs) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/VectorDistance.srv"
  "srv/AddThreeInts.srv"
  DEPENDENCIES std_msgs
)

# Public headers
include_directories(include /usr/include)

# Typesupport target for linking against our generated interfaces
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# ---- Executables ----

# Custom subscriber
add_executable(custom_subscriber src/subscriber_custom_msg.cpp)
target_include_directories(custom_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_subscriber rclcpp std_msgs)
target_link_libraries(custom_subscriber ${cpp_typesupport_target})

# Custom publisher
add_executable(custom_publisher src/publisher_custom_msg.cpp)
target_include_directories(custom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_publisher rclcpp std_msgs)
target_link_libraries(custom_publisher ${cpp_typesupport_target})

# Std subscriber
add_executable(subscriber src/subscriber_std_msg.cpp)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(subscriber rclcpp std_msgs)
target_link_libraries(subscriber ${cpp_typesupport_target})

# Std publisher
add_executable(publisher src/publisher_std_msg.cpp)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(publisher rclcpp std_msgs)
target_link_libraries(publisher ${cpp_typesupport_target})

# Custom server
add_executable(custom_server src/server_custom_msg.cpp)
target_include_directories(custom_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_server rclcpp)
target_link_libraries(custom_server ${cpp_typesupport_target})

# Custom client
add_executable(custom_client src/client_custom_msg.cpp)
target_include_directories(custom_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_client rclcpp)
target_link_libraries(custom_client ${cpp_typesupport_target})

# Std server
add_executable(server src/server_std_msg.cpp)
target_include_directories(server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(server rclcpp)
target_link_libraries(server ${cpp_typesupport_target})

# Std client
add_executable(client src/client_std_msg.cpp)
target_include_directories(client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(client rclcpp)
target_link_libraries(client ${cpp_typesupport_target})

# MoveIt server (Panda’yı hedefe götürür)
add_executable(server_moveit_panda src/server_moveit_panda.cpp)
target_include_directories(server_moveit_panda PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(server_moveit_panda
  rclcpp geometry_msgs moveit_ros_planning_interface std_srvs nlohmann_json)
target_link_libraries(server_moveit_panda ${cpp_typesupport_target})

# JSON’dan hedefleri sırayla gönderen client
add_executable(client_move_targets src/client_move_targets.cpp)
target_include_directories(client_move_targets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(client_move_targets
  rclcpp nlohmann_json std_srvs)
target_link_libraries(client_move_targets ${cpp_typesupport_target})

# ---- Install ----
install(TARGETS
  custom_subscriber
  custom_publisher
  subscriber
  publisher
  custom_server
  custom_client
  server
  client
  server_moveit_panda
  client_move_targets
  DESTINATION lib/${PROJECT_NAME}
)

# Headers + resources (launch/config/scripts)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

# Python GUI script (yürütülebilir olmalı - shebang + +x)
install(PROGRAMS
  scripts/gui_target_to_json_noros.py
  DESTINATION share/${PROJECT_NAME}/scripts
)


# ---- Testing ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- Export ----
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs geometry_msgs moveit_ros_planning_interface std_srvs)
ament_package()
