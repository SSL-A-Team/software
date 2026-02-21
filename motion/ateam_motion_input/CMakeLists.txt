cmake_minimum_required(VERSION 3.8)
project(ateam_motion_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# ateam_controls Rust static library (built via cargo)
set(CONTROLS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../controls")
if((CMAKE_BUILD_TYPE STREQUAL "Release") OR (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  set(CONTROLS_LIB_PATH "${CONTROLS_SRC_DIR}/target/release/libateam_controls_c.a")
else()
  set(CONTROLS_LIB_PATH "${CONTROLS_SRC_DIR}/target/debug/libateam_controls_c.a")
endif()
set(CONTROLS_INCLUDE_DIR "${CONTROLS_SRC_DIR}/ateam-controls-c/include")

add_custom_command(
  OUTPUT ${CONTROLS_LIB_PATH}
  COMMAND cargo build --workspace $<$<CONFIG:Release,RelWithDebInfo>:--release>
  WORKING_DIRECTORY ${CONTROLS_SRC_DIR}
)
add_custom_target(build_ateam_controls ALL DEPENDS ${CONTROLS_LIB_PATH})

add_library(ateam_controls STATIC IMPORTED)
set_target_properties(ateam_controls PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${CONTROLS_INCLUDE_DIR}
  IMPORTED_LOCATION ${CONTROLS_LIB_PATH}
)
add_dependencies(ateam_controls build_ateam_controls)

add_executable(motion_input_node
  src/motion_input_node.cpp
)

ament_target_dependencies(motion_input_node
  "rclcpp"
  "ateam_msgs"
  "tf2"
)
target_link_libraries(motion_input_node ateam_controls nlohmann_json::nlohmann_json)

install(TARGETS
  motion_input_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
