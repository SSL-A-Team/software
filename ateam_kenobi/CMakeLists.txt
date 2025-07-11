cmake_minimum_required(VERSION 3.8)
project(ateam_kenobi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_radio_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ompl REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ateam_spatial REQUIRED)

add_library(kenobi_node_component SHARED
  src/kenobi_node.cpp
)

#
# Core Sources
#
target_sources(kenobi_node_component PRIVATE
  src/core/defense_area_enforcement.cpp
  src/core/in_play_eval.cpp
  src/core/joystick_enforcer.cpp
  src/core/play_selector.cpp
  src/core/path_planning/escape_velocity.cpp
  src/core/path_planning/obstacles.cpp
  src/core/path_planning/path_planner.cpp
  src/core/types/message_conversions.cpp
  src/core/motion/motion_controller.cpp
  src/core/visualization/overlays.cpp
  src/core/play_helpers/easy_move_to.cpp
  src/core/play_helpers/intercept_calculation.cpp
  src/core/play_helpers/lanes.cpp
  src/core/play_helpers/possession.cpp
  src/core/play_helpers/available_robots.cpp
  src/core/play_helpers/robot_assignment.cpp
  src/core/play_helpers/shot_evaluation.cpp
  src/core/play_helpers/window_evaluation.cpp
)

#
# Play Sources
#
target_sources(kenobi_node_component PRIVATE
  src/plays/free_kick_plays/defense/their_free_kick_play.cpp
  src/plays/free_kick_plays/offense/free_kick_on_goal_play.cpp
  src/plays/kickoff_plays/defense/their_kickoff_play.cpp
  src/plays/kickoff_plays/offense/kickoff_on_goal.cpp
  src/plays/kickoff_plays/offense/kickoff_pass_play.cpp
  src/plays/kickoff_plays/offense/our_kickoff_prep_play.cpp
  src/plays/passing_plays/pass_to_lane_play.cpp
  src/plays/passing_plays/pass_to_segment_play.cpp
  src/plays/passing_plays/spatial_pass_play.cpp
  src/plays/stop_plays/default_stop_play.cpp
  src/plays/stop_plays/defensive_stop_play.cpp
  src/plays/stop_plays/offensive_stop_play.cpp
  src/plays/stop_plays/stop_helpers.cpp
  src/plays/test_plays/controls_test_play.cpp
  src/plays/test_plays/spinning_a_play.cpp
  src/plays/test_plays/test_pass_play.cpp
  src/plays/test_plays/test_play.cpp
  src/plays/test_plays/triangle_pass_play.cpp
  src/plays/test_plays/waypoints_play.cpp
  src/plays/halt_play.cpp
  src/plays/kick_on_goal_play.cpp
  src/plays/our_ball_placement_play.cpp
  src/plays/their_ball_placement_play.cpp
  src/plays/wall_play.cpp
  src/plays/basic_122.cpp
  src/plays/our_penalty_play.cpp
  src/plays/their_penalty_play.cpp
  src/plays/defense_play.cpp
  src/plays/extract_play.cpp
)

#
# Tactic Sources
#
target_sources(kenobi_node_component PRIVATE
  src/tactics/blockers.cpp
  src/tactics/defenders.cpp
  src/tactics/multi_move_to.cpp
  src/tactics/pass_to_segment.cpp
  src/tactics/pass.cpp
  src/tactics/standard_defense.cpp
)

#
# Skill Sources
#
target_sources(kenobi_node_component PRIVATE
  src/skills/goalie.cpp
  src/skills/kick_skill.cpp
  src/skills/lane_idler.cpp
  src/skills/line_kick.cpp
  src/skills/pass_receiver.cpp
  src/skills/pivot_kick.cpp
  src/skills/onetouch_kick.cpp
  src/skills/dribble.cpp
  src/skills/extract.cpp
  src/skills/capture.cpp
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
  "ateam_radio_msgs"
  "ateam_msgs"
  "ateam_common"
  "tf2"
  "tf2_geometry_msgs"
  "ateam_geometry"
  "control_toolbox"
  "OMPL"
  "angles"
  OpenCV
  nlohmann_json
  ateam_spatial
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
