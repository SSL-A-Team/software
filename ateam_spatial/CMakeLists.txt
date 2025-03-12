cmake_minimum_required(VERSION 3.8)
project(ateam_spatial)

enable_language(CUDA)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/layers/distance_down_field.cu
  src/layers/distance_from_their_bots.cu
  src/maps/receiver_position_quality.cu
  src/placeholder_cuda.cu
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_STANDARD 17
  CUDA_ARCHITECTURES native
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(CUDAToolkit)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
