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

# --- ADD EXTERNAL DEPENDENCY ---
include(FetchContent)

FetchContent_Declare(
  ateam_controls
  # Use SOURCE_DIR to point to the local folder containing the library's CMakeLists.txt
  # This example assumes ateam_controls is a sibling folder in src/
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../controls"
)

# This downloads/configures the Rust/C wrapper project
FetchContent_MakeAvailable(ateam_controls)
# -------------------------------

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

# Link the non-ROS library
# Note: Ensure 'ateam_controls_c' matches the library name defined 
# in the external project's CMakeLists.txt
target_link_libraries(bangbang_node ateam_controls)

install(TARGETS
  bangbang_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()