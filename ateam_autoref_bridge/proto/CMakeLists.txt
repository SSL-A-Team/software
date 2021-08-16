protobuf_generate_cpp(
  PROTOBUF_SRCS
  PROTOBUF_HDRS
  ssl_gc_api.proto
  ssl_gc_api.proto
  ssl_gc_change.proto
  ssl_gc_ci.proto
  ssl_gc_common.proto
  ssl_gc_engine_config.proto
  ssl_gc_engine.proto
  ssl_gc_game_event.proto
  ssl_gc_geometry.proto
  ssl_gc_rcon_autoref.proto
  ssl_gc_rcon_remotecontrol.proto
  ssl_gc_rcon_team.proto
  ssl_gc_rcon.proto
  ssl_gc_referee_message.proto
  ssl_gc_state.proto
  ssl_vision_detection_tracked.proto
  ssl_vision_detection.proto
  ssl_vision_geometry.proto
  ssl_vision_wrapper_tracked.proto
  ssl_vision_wrapper.proto
)
add_library(league_protobufs SHARED
  ${PROTOBUF_SRCS}
  ${PROTOBUF_HDRS}
)
target_link_libraries(league_protobufs protobuf::libprotobuf)
target_include_directories(league_protobufs INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
install(TARGETS league_protobufs DESTINATION lib)
