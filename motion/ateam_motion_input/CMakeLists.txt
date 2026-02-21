cmake_minimum_required(VERSION 3.8)
project(ateam_motion_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(motion_input_node
  src/motion_input_node.cpp
)

ament_target_dependencies(motion_input_node
  "rclcpp"
  "ateam_msgs"
  "tf2"
)

install(TARGETS
  motion_input_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
