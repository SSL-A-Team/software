cmake_minimum_required(VERSION 3.5)
project(ateam_vision_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(angles REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/filters/interacting_multiple_model_filter.cpp
  src/filters/kalman_filter.cpp
  src/filters/multiple_hypothesis_tracker.cpp
  src/generators/generator_util.cpp
  src/generators/model_input_generator.cpp
  src/generators/transmission_probability_generator.cpp
  src/ateam_vision_filter_node.cpp
  src/camera.cpp
  src/message_conversions.cpp
  src/world.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_common
  ateam_geometry
  ateam_msgs
  ssl_league_msgs
  Eigen3
  tf2
  tf2_geometry_msgs
  angles
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_vision_filter::VisionFilterNode"
  EXECUTABLE ateam_vision_filter_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/filters/interacting_multiple_model_filter_test.cpp
    test/filters/kalman_filter_test.cpp
    test/filters/multiple_hypothesis_tracker_test.cpp
    test/generators/model_input_generator_test.cpp
    test/generators/transmission_probability_generator_test.cpp
    test/camera_test.cpp
    test/world_test.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE test src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test PROPERTIES CXX_STANDARD 17)
  endif()

ament_package()
