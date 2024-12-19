add_library(${PROJECT_NAME}_game_controller_bridge SHARED
    gc_multicast_bridge_node.cpp
    message_conversions.cpp
)
target_include_directories(${PROJECT_NAME}_game_controller_bridge PRIVATE ..)
ament_target_dependencies(${PROJECT_NAME}_game_controller_bridge
  rclcpp
  rclcpp_components
  ssl_ros_bridge_msgs
  ssl_league_msgs
  ssl_league_protobufs
)
target_link_libraries(${PROJECT_NAME}_game_controller_bridge ${PROJECT_NAME}_core)

rclcpp_components_register_node(
  ${PROJECT_NAME}_game_controller_bridge
  PLUGIN "ssl_ros_bridge::game_controller_bridge::GCMulticastBridgeNode"
  EXECUTABLE gc_multicast_bridge_node
)

install(TARGETS ${PROJECT_NAME}_game_controller_bridge DESTINATION lib)
