cmake_minimum_required(VERSION 3.8)
project(ateam_vision_tracker_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ssl_league_protobufs REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(vision_tracker_bridge_component
  src/vision_tracker_bridge_node.cpp
)
target_compile_features(vision_tracker_bridge_component PUBLIC c_std_99 cxx_std_20)
target_compile_options(vision_tracker_bridge_component PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(vision_tracker_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  vision_tracker_bridge_component
  "rclcpp"
  "rclcpp_components"
  "ssl_league_protobufs"
  "ateam_msgs"
  "ateam_common"
  "tf2"
  "tf2_geometry_msgs"
)
rclcpp_components_register_node(
  vision_tracker_bridge_component
  PLUGIN "ateam_vision_tracker_bridge::VisionTrackerBridgeNode"
  EXECUTABLE vision_tracker_bridge_node
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS vision_tracker_bridge_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  vision_tracker_bridge_component
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
