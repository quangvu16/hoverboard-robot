cmake_minimum_required(VERSION 3.8)
project(hoverboard_base_node)

find_package(ament_cmake REQUIRED)

# Configuration and launch only. Yahboom's base_node (odom_raw -> TF) is not
# needed: hoverboard_udp_bridge_node (esp_tcp_server) publishes /odom and TF.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
