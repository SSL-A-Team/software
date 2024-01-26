ament_add_gmock(ateam_geometry_tests
  nearest_points_test.cpp
  normalize_test.cpp
  variant_do_intersect_test.cpp
)
target_link_libraries(ateam_geometry_tests ${PROJECT_NAME} ${PROJECT_NAME}_testing)
