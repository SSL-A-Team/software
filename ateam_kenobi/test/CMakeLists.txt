find_package(ament_cmake_gmock REQUIRED)
ament_add_gtest(ateam_kenobi_tests
  path_planner_test.cpp
)
target_link_libraries(ateam_kenobi_tests ${PROJECT_NAME})