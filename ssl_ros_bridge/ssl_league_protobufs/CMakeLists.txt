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
  proto/ssl_simulation_config.proto
  proto/ssl_simulation_control.proto
  proto/ssl_simulation_error.proto
  proto/ssl_simulation_robot_control.proto
  proto/ssl_simulation_robot_feedback.proto
  proto/ssl_simulation_synchronous.proto
  proto/ssl_vision_detection_tracked.proto
  proto/ssl_vision_detection.proto
  proto/ssl_vision_geometry.proto
  proto/ssl_vision_wrapper_tracked.proto
  proto/ssl_vision_wrapper.proto
)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_HDRS}
  ${PROTOBUF_SRCS}
)
target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
# Disable noisy deprecation warnings we can't control from league protobufs
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
