cmake_minimum_required(VERSION 3.8)
project(ssl_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(ssl_league_protobufs REQUIRED)
find_package(ssl_ros_bridge_msgs REQUIRED)

add_subdirectory(src/core)
add_subdirectory(src/game_controller_bridge)
add_subdirectory(src/team_client)
add_subdirectory(src/vision_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
