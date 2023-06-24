find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
ament_add_gmock(test_segment test_segment.cpp)
target_link_libraries(test_segment ${PROJECT_NAME})

add_compile_options(-g)
