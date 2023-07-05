cmake_minimum_required(VERSION 3.5)
project(ateam_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(angles REQUIRED)
find_package(ssl_league_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/assignment.cpp
  src/bi_directional_udp.cpp
  src/multicast_receiver.cpp
  src/parameters.cpp
  src/status.cpp
  src/game_controller_listener.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  Eigen3
  ateam_msgs
  angles
  ssl_league_msgs
  ateam_geometry
)
target_link_libraries(${PROJECT_NAME}
  Boost::boost
  protobuf::libprotobuf
  ${CMAKE_DL_LIBS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export symbols so we can backtrace
set_property(TARGET ${PROJECT_NAME} PROPERTY ENABLE_EXPORTS ON)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Protobuf angles)

install(
  DIRECTORY include/
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
  add_subdirectory(test)
endif()

ament_package()
