cmake_minimum_required(VERSION 3.8)
project(ateam_software_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Git REQUIRED)

set(SUBMODULE_DIR ${CMAKE_SOURCE_DIR}/software-communication)
set(RADIO_PACKETS_INCLUDE_DIR ${SUBMODULE_DIR}/ateam-common-packets/include)
set(RUST_CRATE_DIR ${SUBMODULE_DIR}/ateam-common-packets/rust-lib)
set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/rs-target)

execute_process(
  COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
  WORKING_DIRECTORY ${SUBMODULE_DIR}
  OUTPUT_VARIABLE SUBMODULE_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _git_result
)
if(NOT _git_result EQUAL 0 OR NOT SUBMODULE_SHA)
  message(FATAL_ERROR "Failed to read git SHA of submodule at ${SUBMODULE_DIR}")
endif()
execute_process(
  COMMAND ${GIT_EXECUTABLE} status --porcelain
  WORKING_DIRECTORY ${SUBMODULE_DIR}
  OUTPUT_VARIABLE _submodule_status
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(_submodule_status)
  set(SUBMODULE_DIRTY true)
else()
  set(SUBMODULE_DIRTY false)
endif()
message(STATUS "software-communication submodule: ${SUBMODULE_SHA} (dirty: ${SUBMODULE_DIRTY})")

execute_process(
  COMMAND ${GIT_EXECUTABLE} rev-parse --absolute-git-dir
  WORKING_DIRECTORY ${SUBMODULE_DIR}
  OUTPUT_VARIABLE _submodule_git_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
file(GLOB_RECURSE _submodule_files ${SUBMODULE_DIR}/*)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${_submodule_git_dir}/HEAD
  ${_submodule_git_dir}/index
  ${_submodule_files}
)

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

ament_package(
  CONFIG_EXTRAS "cmake/ateam_software_communication-extras.cmake.in"
)
