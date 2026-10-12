cmake_minimum_required(VERSION 3.8)
project(esp_tcp_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(esp_tcp_server_node
  src/main.cpp
  src/config.cpp
  src/lenprefix_codec.cpp
  src/tcp_server.cpp
  src/esp_protocol.cpp
  src/ros_bridge.cpp
  src/esp_node.cpp
)

ament_target_dependencies(esp_tcp_server_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  nlohmann_json
  ament_index_cpp
)

# ================= UDP LiDAR receiver node =================
add_executable(udp_lidar_receiver_node
  src/udp_lidar_receiver.cpp
)

ament_target_dependencies(udp_lidar_receiver_node
  rclcpp
  std_msgs
  sensor_msgs
)

# ================= Hoverboard UDP bridge node =================
add_executable(hoverboard_udp_bridge_node
  src/hoverboard_udp_bridge.cpp
)

ament_target_dependencies(hoverboard_udp_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  esp_tcp_server_node
  udp_lidar_receiver_node
  hoverboard_udp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
