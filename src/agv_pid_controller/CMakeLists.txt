
cmake_minimum_required(VERSION 3.8)
project(agv_pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pid_velocity_node
  src/pid_velocity_node.cpp
)

target_compile_features(pid_velocity_node PUBLIC cxx_std_17)

ament_target_dependencies(pid_velocity_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  control_toolbox
  tf2
  tf2_ros
)

install(TARGETS pid_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()