cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Arayüz (srv) dosyasını tanımlıyoruz
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Server.srv"
  DEPENDENCIES geometry_msgs
)

# İSTEMCİYİ DERLE
add_executable(yilmaz_client src/yilmaz_client.cpp)
ament_target_dependencies(yilmaz_client rclcpp geometry_msgs)

# Eski komut:
# rosidl_target_interfaces(yilmaz_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Yeni komutlar:
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(yilmaz_client "${cpp_typesupport_target}")

install(TARGETS
  yilmaz_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
