cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Cài đặt các thư mục tài nguyên để ROS 2 nhận diện được
install(
  DIRECTORY urdf launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY urdf launch meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/keyboard_direct.py
    scripts/web_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()