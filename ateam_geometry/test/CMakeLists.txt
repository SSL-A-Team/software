find_package(ament_cmake_gmock REQUIRED)
ament_add_gmock(ateam_geometry_tests
  variant_do_intersect_test.cpp
)
target_link_libraries(ateam_geometry_tests ${PROJECT_NAME})
