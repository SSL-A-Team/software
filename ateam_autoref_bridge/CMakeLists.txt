cmake_minimum_required(VERSION 3.5)
project(ateam_autoref_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED)

add_subdirectory(proto)

add_library(${PROJECT_NAME} SHARED
  src/autoref_bridge_node.cpp
  src/message_conversions.cpp
  src/referee_multicast_bridge.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ssl_league_msgs
)
target_link_libraries(${PROJECT_NAME}
  league_protobufs 
  Boost::boost
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_autoref_bridge::AutorefBridgeNode"
  EXECUTABLE autoref_bridge_node
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
