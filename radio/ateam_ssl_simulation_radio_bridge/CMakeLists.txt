cmake_minimum_required(VERSION 3.5)
project(ateam_ssl_simulation_radio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_radio_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ssl_league_protobufs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ssl_simulation_radio_bridge_node.cpp
  src/message_conversions.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_common
  ateam_msgs
  ateam_radio_msgs
  tf2
  tf2_geometry_msgs
  ssl_league_protobufs
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode"
  EXECUTABLE ssl_simulation_radio_bridge_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
