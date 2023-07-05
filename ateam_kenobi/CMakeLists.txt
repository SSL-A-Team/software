cmake_minimum_required(VERSION 3.8)
project(ateam_kenobi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ompl REQUIRED)

add_library(kenobi_node_component SHARED
  src/kenobi_node.cpp
  src/play_selector.cpp
  src/path_planning/path_planner.cpp
  src/plays/test_play.cpp
  src/plays/halt_play.cpp
  src/plays/our_kickoff_play.cpp
  src/plays/wall_play.cpp
  src/types/message_conversions.cpp
  src/motion/motion_controller.cpp
  src/visualization/overlay_publisher.cpp
)
target_include_directories(kenobi_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(kenobi_node_component PUBLIC cxx_std_20)
ament_target_dependencies(
  kenobi_node_component
  "rclcpp"
  "rclcpp_components"
  "ateam_msgs"
  "ateam_common"
  "tf2"
  "tf2_geometry_msgs"
  "ateam_geometry"
  "control_toolbox"
  "OMPL"
)
rclcpp_components_register_node(
  kenobi_node_component
  PLUGIN "ateam_kenobi::KenobiNode"
  EXECUTABLE ateam_kenobi_node
)

install(TARGETS kenobi_node_component DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
