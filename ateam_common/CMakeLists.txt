cmake_minimum_required(VERSION 3.5)
project(ateam_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(angles REQUIRED)
find_package(ssl_league_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bi_directional_udp.cpp
  src/cache_directory.cpp
  src/get_ip_addresses.cpp
  src/multicast_receiver.cpp
  src/game_controller_listener.cpp
  src/km_assignment.cpp
  src/time.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ateam_geometry
  ateam_msgs
  angles
  ssl_league_msgs
  Eigen3
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
ament_export_dependencies(rclcpp ateam_geometry ateam_msgs angles ssl_league_msgs Eigen3 Boost Protobuf)

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
