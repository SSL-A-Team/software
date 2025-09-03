find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gmock(test_protobuf_logging test_protobuf_logging.cpp)
target_link_libraries(test_protobuf_logging ${PROJECT_NAME})

ament_add_gtest(test_geometry test_angle.cpp)
target_link_libraries(test_geometry ${PROJECT_NAME})

ament_add_gmock(test_km_assignment test_km_assignment.cpp)
target_link_libraries(test_km_assignment ${PROJECT_NAME})