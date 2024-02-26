cmake_minimum_required(VERSION 3.5)
project(ateam_data_conversion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_ssl_vision_bridge REQUIRED)
find_package(ateam_game_controller_bridge REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ssl_log_reader.cpp
  src/ssl_log_replayer_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(FIND_DEPS
  rclcpp
  rclcpp_components
  # ateam_common
  # ateam_msgs
  # ssl_league_msgs
  # ssl_league_protobufs
  ateam_ssl_vision_bridge
  ateam_game_controller_bridge
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  ${FIND_DEPS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_data_conversion::SSLLogReplayerNode"
  EXECUTABLE ssl_log_replayer_node
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${FIND_DEPS})

ament_package()
