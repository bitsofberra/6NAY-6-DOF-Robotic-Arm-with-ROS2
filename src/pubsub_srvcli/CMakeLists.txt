cmake_minimum_required(VERSION 3.8)
project(pubsub_srvcli)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/VectorDistance.srv"
  "srv/AddThreeInts.srv"
  DEPENDENCIES std_msgs
)

include_directories(include /usr/include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)



# Custom subscriber
add_executable(custom_subscriber src/subscriber_custom_msg.cpp)
target_include_directories(custom_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_subscriber rclcpp std_msgs)

# Custom publisher
add_executable(custom_publisher src/publisher_custom_msg.cpp)
target_include_directories(custom_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(custom_publisher rclcpp std_msgs)

# Std subscriber
add_executable(subscriber src/subscriber_std_msg.cpp)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(subscriber rclcpp std_msgs)

# Std publisher
add_executable(publisher src/publisher_std_msg.cpp)
target_include_directories(publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(publisher rclcpp std_msgs)


# Custom server
add_executable(custom_server src/server_custom_msg.cpp)
ament_target_dependencies(custom_server rclcpp)
target_link_libraries(custom_server ${cpp_typesupport_target})

# Custom client
add_executable(custom_client src/client_custom_msg.cpp)
ament_target_dependencies(custom_client rclcpp)
target_link_libraries(custom_client ${cpp_typesupport_target})

# Std server
add_executable(server src/server_std_msg.cpp)
ament_target_dependencies(server rclcpp)
target_link_libraries(server ${cpp_typesupport_target})

# Std client
add_executable(client src/client_std_msg.cpp)
ament_target_dependencies(client rclcpp)
target_link_libraries(client ${cpp_typesupport_target})



install(TARGETS
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


# Test 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)
ament_package()
