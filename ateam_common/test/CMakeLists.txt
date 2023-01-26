find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gmock(test_protobuf_logging test_protobuf_logging.cpp)
target_link_libraries(test_protobuf_logging ${PROJECT_NAME})

add_compile_options(-g)
ament_add_gtest(test_status test_status.cpp)
target_link_libraries(test_status ${PROJECT_NAME})
set_property(TARGET test_status PROPERTY ENABLE_EXPORTS ON)

ament_add_gmock(test_logging test_logging.cpp)
set_target_properties(test_logging PROPERTIES CXX_STANDARD 17)
target_link_libraries(test_logging ${PROJECT_NAME})

ament_add_gtest(test_parameters test_parameters.cpp)
target_link_libraries(test_parameters ${PROJECT_NAME})
