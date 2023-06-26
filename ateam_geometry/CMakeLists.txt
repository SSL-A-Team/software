cmake_minimum_required(VERSION 3.8)
project(ateam_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/circle.cpp
  src/rectangle.cpp
  src/segment.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  Eigen3
  ateam_common
  ateam_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY ENABLE_EXPORTS ON)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

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
  # Will add this subdirectory once I've written tests
  add_subdirectory(test)
endif()

ament_package()
