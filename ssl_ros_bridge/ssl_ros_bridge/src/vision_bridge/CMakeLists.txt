add_library(${PROJECT_NAME}_vision_bridge SHARED
  ssl_vision_bridge_node.cpp
  message_conversions.cpp
)
target_include_directories(${PROJECT_NAME}_vision_bridge PRIVATE ..)
ament_target_dependencies(${PROJECT_NAME}_vision_bridge
  rclcpp
  rclcpp_components
  ssl_league_msgs
  ssl_league_protobufs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_vision_bridge ${PROJECT_NAME}_core)

rclcpp_components_register_node(
  ${PROJECT_NAME}_vision_bridge
  PLUGIN "ssl_ros_bridge::vision_bridge::SSLVisionBridgeNode"
  EXECUTABLE vision_bridge_node
)

install(TARGETS ${PROJECT_NAME}_vision_bridge DESTINATION lib)
