cmake_minimum_required(VERSION 3.8)
project(ateam_path_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ateam_geometry REQUIRED)
find_package(ateam_common REQUIRED)
find_package(ateam_game_state REQUIRED)

add_subdirectory(ateam_controls)
if(NOT TARGET ateam_controls)
  message(FATAL_ERROR "Failed to build or find ateam_controls library")
endif()

add_library(${PROJECT_NAME} SHARED
  src/planner.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}
  ateam_geometry
  ateam_common
  ateam_game_state
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ateam_controls
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ateam_path_planning
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
