cmake_minimum_required(VERSION 3.8)
project(new_super_vision_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_common REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/vision_filter_node.cpp
  src/camera.cpp
  src/filtered_robot.cpp
  src/filtered_ball.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/kalman/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ateam_msgs
  ateam_common
  ssl_league_msgs
  Eigen3
)
target_link_libraries(${PROJECT_NAME}
  Boost::boost
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "new_super_vision::VisionFilterNode"
  EXECUTABLE ateam_new_super_vision
)
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    "kalman/*"
    "kalman/cmake/*"
  )
  file(GLOB_RECURSE ALL_CMAKE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
  )

  list(FILTER ALL_CMAKE_FILES EXCLUDE REGEX
    ".*/kalman/.*"
  )

  ament_lint_cmake(${ALL_CMAKE_FILES})
  add_subdirectory(test)
endif()

ament_package()
