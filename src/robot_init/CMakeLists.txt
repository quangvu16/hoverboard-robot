cmake_minimum_required(VERSION 3.8)
project(robot_init)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(angles REQUIRED)


include_directories(include)

add_executable(robot_init
  src/main.cpp
  src/robot.cpp
  src/robot_status.cpp
  src/robot_function.cpp
  src/robot_task.cpp
  src/robot_slam.cpp
  src/robot_protocol.cpp
  src/robot_netClient.cpp
  src/robot_publish.cpp
  src/robot_nav.cpp
  src/robot_error.cpp
)
ament_target_dependencies(robot_init
  rclcpp
  std_msgs
  nlohmann_json
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  nav2_msgs
  angles
  robot_msgs
)
target_link_libraries(robot_init nlohmann_json::nlohmann_json)

install(TARGETS
  robot_init
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

