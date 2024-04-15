ament_add_gmock(ateam_geometry_tests
  do_intersect_test.cpp
  nearest_points_test.cpp
  normalize_test.cpp
  testing_utils_test.cpp
)
target_link_libraries(ateam_geometry_tests ${PROJECT_NAME} ${PROJECT_NAME}_testing)
