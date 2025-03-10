find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(ateam_spatial_tests
  gpu_array_tests.cpp
  gpu_object_tests.cpp
)

target_include_directories(ateam_spatial_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)

target_link_libraries(ateam_spatial_tests
  ${PROJECT_NAME}
)
