add_library(${PROJECT_NAME}_tame_client SHARED
    team_client_node.cpp
    team_client.cpp
)
target_include_directories(${PROJECT_NAME}_tame_client PRIVATE ..)
ament_target_dependencies(${PROJECT_NAME}_tame_client
  rclcpp
  rclcpp_components
  ssl_ros_bridge_msgs
  ssl_league_msgs
  ssl_league_protobufs
)
target_link_libraries(${PROJECT_NAME}_tame_client ${PROJECT_NAME}_core)

rclcpp_components_register_node(
  ${PROJECT_NAME}_tame_client
  PLUGIN "ssl_ros_bridge::game_controller_bridge::TeamClientNode"
  EXECUTABLE team_client_node
)

install(TARGETS ${PROJECT_NAME}_tame_client DESTINATION lib)

