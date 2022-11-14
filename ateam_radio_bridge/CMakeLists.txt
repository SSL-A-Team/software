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
  src/radio_bridge_node.cpp
  src/rnp_packet_helpers.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME} PRIVATE software-communication/ateam-common-packets/include)
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
  # list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  # find_package(ament_lint_auto REQUIRED)
  # Disable auto finding of cpplint so we can customize its arguments
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # set(ament_cmake_cpplint_FOUND FALSE)
  # find_package(ament_cmake_cpplint)
  # ament_cpplint(EXCLUDE software-communication)

  add_subdirectory(test)
endif()

ament_package()
