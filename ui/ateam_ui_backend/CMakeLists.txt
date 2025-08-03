cmake_minimum_required(VERSION 3.5)
project(ateam_ui_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ssl_ros_bridge_msgs REQUIRED)
find_package(ateam_radio_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ateam_ui_backend_node.cpp
  src/message_conversions.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME}
  Boost::boost
  ${CMAKE_DL_LIBS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_common
  ateam_msgs
  ssl_ros_bridge_msgs
  ateam_radio_msgs
  std_srvs
  ssl_league_msgs
  nlohmann_json
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ateam_ui_backend_node::AteamUIBackendNode"
  EXECUTABLE ateam_ui_backend_node
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
