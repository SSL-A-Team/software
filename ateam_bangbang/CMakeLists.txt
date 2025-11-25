cmake_minimum_required(VERSION 3.8)
project(ateam_bangbang)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_radio_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_game_state REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ompl REQUIRED)
find_package(angles REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(bangbang_node
  src/bangbang_node.cpp
)

ament_target_dependencies(bangbang_node
  "rclcpp"
  "rclcpp_components"
  "ateam_radio_msgs"
  "ateam_msgs"
  "ateam_common"
  "ateam_game_state"
  "tf2"
  "tf2_geometry_msgs"
  "ateam_geometry"
  "control_toolbox"
  "OMPL"
  "angles"
  nlohmann_json
)

install(TARGETS
  bangbang_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()