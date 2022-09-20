cmake_minimum_required(VERSION 3.5)
project(ateam_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ateam_ai_node.cpp
  src/behavior/behavior_evaluator.cpp
  src/behavior/behavior_evaluator.hpp
  src/behavior/behavior_executor.cpp
  src/behavior/behavior_executor.hpp
  src/behavior/behavior_feedback.hpp
  src/behavior/behavior_follower.cpp
  src/behavior/behavior_follower.hpp
  src/behavior/behavior_realization.cpp
  src/behavior/behavior_realization.hpp
  src/behavior/behavior.hpp
  src/trajectory_generation/trajectory_editor.cpp
  src/trajectory_generation/trajectory_editor.hpp
  src/trajectory_generation/trajectory_generation.cpp
  src/trajectory_generation/trajectory_generation.hpp
  src/trajectory_generation/trapezoidal_motion_profile.cpp
  src/trajectory_generation/trapezoidal_motion_profile.hpp
  src/types/ball.hpp
  src/types/field.hpp
  src/types/referee_info.hpp
  src/types/robot.hpp
  src/types/world.hpp
  src/util/directed_graph.hpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_msgs
  Eigen3
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_ai::ATeamAINode"
  EXECUTABLE ateam_ai_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/trajectory_generation/trajectory_editor_test.cpp
    test/trajectory_generation/trapezoidal_motion_profile_test.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE test src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test PROPERTIES CXX_STANDARD 20)
endif()

ament_package()
