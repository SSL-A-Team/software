cmake_minimum_required(VERSION 3.8)
project(ateam_software_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(SUBMODULE_DIR ${CMAKE_SOURCE_DIR}/software-communication)
set(RADIO_PACKETS_INCLUDE_DIR ${SUBMODULE_DIR}/ateam-common-packets/include)
set(RUST_CRATE_DIR ${SUBMODULE_DIR}/ateam-common-packets/rust-lib)
set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/rs-target)
file(GLOB_RECURSE PACKET_HEADERS ${RADIO_PACKETS_INCLUDE_DIR}/*)
list(APPEND RUST_LIB_SOURCES
  ${RUST_CRATE_DIR}/Cargo.toml
  ${RUST_CRATE_DIR}/Cargo.lock
  ${RUST_CRATE_DIR}/build.rs
  ${RUST_CRATE_DIR}/src/lib.rs
  ${RUST_CRATE_DIR}/src/radio.rs
)
set(RUST_LIB_OUTPUT ${RUST_TARGET_DIR}/$<IF:$<CONFIG:Release,RelWithDebInfo>,release,debug>/libateam_common_packets.rlib)
add_custom_command(
  OUTPUT ${RUST_LIB_OUTPUT}
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR} $<$<CONFIG:RelWithDebInfo,Debug>:RUSTFLAGS=-g> --unset=MAKEFLAGS --unset=MFLAGS cargo build $<$<CONFIG:Release,RelWithDebInfo>:--release> --no-default-features
  WORKING_DIRECTORY ${RUST_CRATE_DIR}
  DEPENDS ${RUST_LIB_SOURCES} ${PACKET_HEADERS}
)
add_custom_target(build_rust_lib ALL
  DEPENDS ${RUST_LIB_OUTPUT}
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${RADIO_PACKETS_INCLUDE_DIR}/
  DESTINATION include/ateam_radio_msgs/ateam_radio_msgs/packets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
