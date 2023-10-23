cmake_minimum_required(VERSION 3.5)
project(ateam_data_conversion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(ssl_league_protobufs REQUIRED)
find_package(ateam_vision_filter REQUIRED)
find_package(ateam_ssl_vision_bridge REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ssl_log_replayer.cpp
  src/ssl_to_rosbag_node.cpp
  src/ssl_to_rosbag_cli.cpp
  src/ssl_to_rosbag.cpp
  src/ssl_vision_adapter.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_common
  ateam_msgs
  ssl_league_msgs
  ssl_league_protobufs
  ateam_vision_filter
  ateam_ssl_vision_bridge
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_data_conversion::SslToRosbagNode"
  EXECUTABLE ssl_to_rosbag_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test
#   )
#   target_include_directories(${PROJECT_NAME}_test PRIVATE test src)
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#   set_target_properties(${PROJECT_NAME}_test PROPERTIES CXX_STANDARD 17)
# endif()

ament_package()
