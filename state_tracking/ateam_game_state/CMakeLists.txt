cmake_minimum_required(VERSION 3.8)
project(ateam_game_state)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_radio_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/type_adapters.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  ${PROJECT_NAME}
  ateam_common
  ateam_geometry
  ateam_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)


add_library(game_state_tracker_component SHARED
  src/game_state_tracker/game_state_tracker.cpp
  src/game_state_tracker/double_touch_evaluator.cpp
  src/game_state_tracker/in_play_evaluator.cpp
)
target_include_directories(game_state_tracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(game_state_tracker_component PUBLIC cxx_std_20)
target_compile_options(game_state_tracker_component PRIVATE -Werror -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  game_state_tracker_component
  rclcpp
  rclcpp_components
  ateam_common
  ateam_geometry
  ateam_msgs
  ateam_radio_msgs
  ssl_league_msgs
)
target_link_libraries(game_state_tracker_component ${PROJECT_NAME})
rclcpp_components_register_node(
  game_state_tracker_component
  PLUGIN "ateam_game_state::GameStateTracker"
  EXECUTABLE game_state_tracker_node
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(tf2 tf2_geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS game_state_tracker_component DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
