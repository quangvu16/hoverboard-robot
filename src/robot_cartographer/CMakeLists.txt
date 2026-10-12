cmake_minimum_required(VERSION 3.8)
project(robot_cartographer)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/initialpose_relay.py scripts/cartographer_odom.py scripts/save_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
