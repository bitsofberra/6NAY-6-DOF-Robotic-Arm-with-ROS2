cmake_minimum_required(VERSION 3.10)
project(moveit LANGUAGES CXX C)

# Derleme standartları
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Paketler
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)                 # <-- eklendi
find_package(control_msgs REQUIRED)                  # <-- eklendi (GripperCommand)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)                   # <-- eklendi (trajectory_processing, robot_trajectory)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)




# Qt GUI
find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

# ---------------- SRV ----------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VectorDistance.srv"
)
# Typesupport hedefi
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# -------------- custom_server --------------
add_executable(custom_server src/moveit_server.cpp)
ament_target_dependencies(custom_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  std_srvs 
)
target_link_libraries(custom_server "${cpp_typesupport_target}")
add_dependencies(custom_server ${PROJECT_NAME}__rosidl_generator_cpp)
install(TARGETS custom_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ----------- planning_scene_relay -----------
add_executable(planning_scene_relay src/planning_scene_relay.cpp)
ament_target_dependencies(planning_scene_relay
  rclcpp
  moveit_msgs
)
install(TARGETS planning_scene_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------------- pick_place_node ----------------
add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node
  rclcpp
  rclcpp_action            # <-- eklendi
  control_msgs             # <-- eklendi
  geometry_msgs
  moveit_core              # <-- eklendi (IPTP/RobotTrajectory)
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  tf2_geometry_msgs
  sensor_msgs

  
)

install(TARGETS pick_place_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -------------- custom_client --------------
add_executable(custom_client src/moveit_client.cpp)
ament_target_dependencies(custom_client
  rclcpp
  geometry_msgs
)
target_link_libraries(custom_client
  nlohmann_json::nlohmann_json
  "${cpp_typesupport_target}"
)
add_dependencies(custom_client ${PROJECT_NAME}__rosidl_generator_cpp)
install(TARGETS custom_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ------------- vector_input_gui -------------
add_executable(vector_input_gui src/vector_input_gui.cpp)
ament_target_dependencies(vector_input_gui
  rclcpp
  geometry_msgs
)
target_link_libraries(vector_input_gui Qt5::Widgets)
install(TARGETS vector_input_gui
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Install / Export ----------------
install(DIRECTORY srv/     DESTINATION share/${PROJECT_NAME}/srv)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/  DESTINATION share/${PROJECT_NAME}/urdf)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
