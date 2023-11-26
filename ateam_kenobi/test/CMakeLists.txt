find_package(ament_cmake_gmock REQUIRED)

set(CGAL_DATA_DIR ".") # Silence CGAL warning since we don't use its data directory
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL REQUIRED)

ament_add_gmock(ateam_kenobi_tests
  path_planner_test.cpp
)

target_include_directories(ateam_kenobi_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

ament_target_dependencies(
  ateam_kenobi_tests
  "rclcpp"
  "rclcpp_components"
  "ateam_msgs"
  "ateam_common"
  "ssl_league_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "ateam_geometry"
  "control_toolbox"
  "OMPL"
)

target_link_libraries(ateam_kenobi_tests ${PROJECT_NAME})