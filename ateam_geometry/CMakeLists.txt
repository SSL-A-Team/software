cmake_minimum_required(VERSION 3.8)
project(ateam_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

set(CGAL_DATA_DIR ".") # Silence CGAL warning since we don't use its data directory
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/do_intersect.cpp
  src/intersection.cpp
  src/nearest_point.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  angles
  Eigen3
  CGAL
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(CGAL angles)

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
  find_package(ament_cmake_gmock REQUIRED)
  add_library(${PROJECT_NAME}_testing INTERFACE)
  target_include_directories(${PROJECT_NAME}_testing INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(${PROJECT_NAME}_testing PROPERTIES CXX_STANDARD 20)
  target_link_libraries(${PROJECT_NAME}_testing INTERFACE ${PROJECT_NAME})
  ament_export_targets(${PROJECT_NAME}_testing HAS_LIBRARY_TARGET)
  install(TARGETS ${PROJECT_NAME}_testing
          EXPORT ${PROJECT_NAME}_testing
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES DESTINATION include)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
