cmake_minimum_required(VERSION 3.5)
project(ssl_league_protobufs)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(
  PROTOBUF_SRCS
  PROTOBUF_HDRS
  proto/ssl_gc_api.proto
  proto/ssl_gc_api.proto
  proto/ssl_gc_change.proto
  proto/ssl_gc_ci.proto
  proto/ssl_gc_common.proto
  proto/ssl_gc_engine_config.proto
  proto/ssl_gc_engine.proto
  proto/ssl_gc_game_event.proto
  proto/ssl_gc_geometry.proto
  proto/ssl_gc_rcon_autoref.proto
  proto/ssl_gc_rcon_remotecontrol.proto
  proto/ssl_gc_rcon_team.proto
  proto/ssl_gc_rcon.proto
  proto/ssl_gc_referee_message.proto
  proto/ssl_gc_state.proto
  proto/ssl_vision_detection_tracked.proto
  proto/ssl_vision_detection.proto
  proto/ssl_vision_geometry.proto
  proto/ssl_vision_wrapper_tracked.proto
  proto/ssl_vision_wrapper.proto
)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_SRCS}
  ${PROTOBUF_HDRS}
)
target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)
target_include_directories(${PROJECT_NAME} INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR}
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
