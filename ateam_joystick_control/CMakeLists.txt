cmake_minimum_required(VERSION 3.8)
project(ateam_joystick_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joystick_control_node.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_msgs
  sensor_msgs
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_joystick_control::JoystickControlNode"
  EXECUTABLE joystick_control_node
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
