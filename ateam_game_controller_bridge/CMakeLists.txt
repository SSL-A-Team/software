cmake_minimum_required(VERSION 3.5)
project(ateam_game_controller_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(ssl_league_protobufs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gc_multicast_bridge_node.cpp
  src/message_conversions.cpp
  src/team_client_node.cpp
  src/team_client.cpp
)

# std cmake relocatable package practice stuff
# apparently for colocn supposed to put this even another directory down?
# https://colcon.readthedocs.io/en/released/user/overriding-packages.html#id21
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(FIND_DEPS
  rclcpp
  rclcpp_components
  ateam_common
  ateam_msgs
  ssl_league_msgs
  ssl_league_protobufs
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  ${FIND_DEPS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_game_controller_bridge::GCMulticastBridgeNode"
  EXECUTABLE gc_multicast_bridge_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_game_controller_bridge::TeamClientNode"
  EXECUTABLE team_client_node
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
