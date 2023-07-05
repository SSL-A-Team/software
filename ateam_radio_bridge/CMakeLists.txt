cmake_minimum_required(VERSION 3.8)
project(ateam_radio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_common REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/conversion.cpp
  src/ip_address_helpers.cpp
  src/radio_bridge_node.cpp
  src/rnp_packet_helpers.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
target_include_directories(${PROJECT_NAME} PUBLIC software-communication/ateam-common-packets/include)
target_include_directories(${PROJECT_NAME} PUBLIC src)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_msgs
  ateam_common
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_radio_bridge::RadioBridgeNode"
  EXECUTABLE radio_bridge_node
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  # TODO(barulicm) Linters are omitted from this package because the software-communication repo
  # doesn't respect our style rules and setting exclusion rules for ament linters doesn't seem
  # to actually work in Humble

  add_subdirectory(test)
endif()

ament_package()
