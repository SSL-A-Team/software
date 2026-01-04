find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(filter_test filter_test.cpp)
target_link_libraries(filter_test ${PROJECT_NAME})
