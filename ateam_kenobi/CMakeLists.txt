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
find_package(angles REQUIRED)

add_library(kenobi_node_component SHARED
  src/defense_area_enforcement.cpp
  src/in_play_eval.cpp
  src/kenobi_node.cpp
  src/play_selector.cpp
  src/path_planning/obstacles.cpp
  src/path_planning/path_planner.cpp
  src/plays/passing_plays/pass_to_segment_play.cpp
  src/plays/test_play.cpp
  src/plays/halt_play.cpp
  src/plays/kick_on_goal_play.cpp
  src/plays/stop_play.cpp
  src/plays/our_ball_placement_play.cpp
  src/plays/their_ball_placement_play.cpp
  src/plays/our_kickoff_play.cpp
  src/plays/wall_play.cpp
  src/plays/basic_122.cpp
  src/plays/our_penalty_play.cpp
  src/plays/their_kickoff_play.cpp
  src/plays/their_penalty_play.cpp
  src/plays/controls_test_play.cpp
  src/plays/triangle_pass_play.cpp
  src/plays/waypoints_play.cpp
  src/plays/spinning_a_play.cpp
  src/types/message_conversions.cpp
  src/motion/motion_controller.cpp
  src/visualization/overlays.cpp
  src/play_helpers/easy_move_to.cpp
  src/play_helpers/lanes.cpp
  src/play_helpers/available_robots.cpp
  src/play_helpers/robot_assignment.cpp
  src/play_helpers/window_evaluation.cpp
  src/skills/goalie.cpp
  src/skills/kick_skill.cpp
  src/skills/lane_idler.cpp
  src/skills/line_kick.cpp
  src/skills/pass_receiver.cpp
  src/skills/pivot_kick.cpp
  src/skills/dribble.cpp
  src/skills/capture.cpp
  src/tactics/blockers.cpp
  src/tactics/defenders.cpp
  src/tactics/pass_to_segment.cpp
  src/tactics/pass.cpp
  src/tactics/standard_defense.cpp
)
target_include_directories(kenobi_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(kenobi_node_component PUBLIC cxx_std_20)
target_compile_options(kenobi_node_component PRIVATE -Werror -Wall -Wextra -Wpedantic)
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
  "angles"
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
  add_subdirectory(test)
endif()

ament_package()
