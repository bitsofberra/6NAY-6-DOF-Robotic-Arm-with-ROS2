cmake_minimum_required(VERSION 3.8)
project(tutorial_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Paket bağımlılıkları
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES builtin_interfaces
)

ament_target_dependencies(pub rclcpp std_msgs)

# Executable: berra
add_executable(pub src/costum_pub.cpp)

ament_target_dependencies(sub rclcpp std_msgs)

# Executable: berra
add_executable(sub src/costum_sub.cpp)