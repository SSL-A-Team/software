cmake_minimum_required(VERSION 3.5)
project(ateam_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ateam_ai_node.cpp
  src/behavior/behavior_evaluator.hpp
  src/behavior/behavior_executor.hpp
  src/behavior/behavior_feedback.hpp
  src/behavior/behavior_realization.hpp
  src/behavior/behavior.hpp
  src/trajectory_generation/trajectory_generation.hpp
  src/util/directed_graph.hpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  Eigen3
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_ai::ATeamAINode"
  EXECUTABLE ai_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
