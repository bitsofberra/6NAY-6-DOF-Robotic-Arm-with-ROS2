cmake_minimum_required(VERSION 3.8)
project(panda_yolo_pickplace)

# Qt için MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Core
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED) 
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)


# Vision
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# TF / Msgs
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# MoveIt
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# GUI / Plugin
find_package(rqt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Interfaces (srv) — INSTALL ETMEDEN üret
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TriggerPickPlace.srv"
  DEPENDENCIES std_msgs geometry_msgs
  SKIP_INSTALL
)

ament_export_dependencies(rosidl_default_runtime)

# (Varsa) başlıklar
include_directories(include)

# ===== Executables & Library =====
add_executable(robot_description_publisher src/robot_description_publisher.cpp)
ament_target_dependencies(robot_description_publisher rclcpp std_msgs ament_index_cpp)

add_executable(color_detector_node src/color_detector_node.cpp)
ament_target_dependencies(color_detector_node
  rclcpp image_transport cv_bridge OpenCV
  tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs
)

add_executable(pick_place_server src/pick_place_server.cpp)
ament_target_dependencies(pick_place_server
  rclcpp moveit_ros_planning_interface geometry_msgs
  moveit_msgs shape_msgs
)

add_executable(panda_arm_commander src/panda_arm_commander.cpp)
ament_target_dependencies(panda_arm_commander rclcpp std_msgs std_srvs trajectory_msgs)


add_executable(demo_pick_place src/demo_pick_place.cpp)
ament_target_dependencies(demo_pick_place rclcpp trajectory_msgs)

add_library(panda_gui SHARED src/gui_panel.cpp)
ament_target_dependencies(panda_gui
  rqt_gui_cpp rclcpp pluginlib
)
target_link_libraries(panda_gui Qt5::Widgets)

# Üretilen srv başlıklarına derleme sırası bağımlılığı
add_dependencies(pick_place_server ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(pick_place_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

add_dependencies(panda_gui ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(panda_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Modern typesupport link (deprecation uyarısını kaldırır)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(pick_place_server "${cpp_typesupport_target}")
target_link_libraries(panda_gui       "${cpp_typesupport_target}")

# rqt plugin açıklaması
pluginlib_export_plugin_description_file(rqt_gui_cpp plugin_description.xml)

# Install
install(TARGETS color_detector_node pick_place_server panda_gui robot_description_publisher demo_pick_place panda_arm_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf world rviz resource 
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rqt_gui_cpp pluginlib)
ament_package()
