ament_add_gmock(ateam_geometry_tests
  angles_test.cpp
  do_intersect_test.cpp
  intersection_test.cpp
  nearest_point_test.cpp
  normalize_test.cpp
  testing_utils_test.cpp
)
target_link_libraries(ateam_geometry_tests ${PROJECT_NAME} ${PROJECT_NAME}_testing)
