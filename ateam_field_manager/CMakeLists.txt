cmake_minimum_required(VERSION 3.5)
project(ateam_field_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ateam_field_manager_node.cpp
  src/message_conversions.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_common
  ateam_msgs
  ssl_league_msgs
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_field_manager::FieldManagerNode"
  EXECUTABLE ateam_field_manager_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
