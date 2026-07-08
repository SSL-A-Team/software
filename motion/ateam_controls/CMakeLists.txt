cmake_minimum_required(VERSION 3.8)
project(ateam_controls)

find_package(ament_cmake REQUIRED)
find_package(ateam_game_state REQUIRED)
find_package(ateam_geometry REQUIRED)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/rs-target)
add_subdirectory(controls)

add_library(
  ateam_controls_cpp SHARED
  src/exceptions.cpp
  src/predict.cpp
)
target_compile_features(ateam_controls_cpp PUBLIC cxx_std_20)
target_compile_options(ateam_controls_cpp PRIVATE -Werror -Wall -Wextra -Wpedantic)
target_link_libraries(ateam_controls_cpp PUBLIC
  ateam_game_state::ateam_game_state
  ateam_geometry::ateam_geometry
)
target_link_libraries(ateam_controls_cpp PRIVATE
  ateam_controls
)
target_include_directories(ateam_controls_cpp
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/controls/*)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/controls)

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(EXCLUDE ${AMENT_LINT_AUTO_FILE_EXCLUDE})

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(EXCLUDE ${AMENT_LINT_AUTO_FILE_EXCLUDE})

  # TODO(barulicm): Excludes not working right. Consider adding AMENT_IGNORE file to controls repo
  # find_package(ament_cmake_copyright REQUIRED)
  # ament_copyright(EXCLUDE ${AMENT_LINT_AUTO_FILE_EXCLUDE})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
endif()

install(
  DIRECTORY controls/ateam-controls-c/include/
  DESTINATION include
)
install(
  FILES $<TARGET_FILE:ateam_controls>
  DESTINATION lib
)

ament_export_targets(ateam_controls_cpp HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ateam_controls_cpp
        EXPORT ateam_controls_cpp
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

ament_export_dependencies(ateam_game_state ateam_geometry)

_ament_cmake_export_libraries_register_environment_hook()
_ament_cmake_export_libraries_register_package_hook()
list(APPEND _AMENT_EXPORT_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libateam_controls_c.a)

ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include)

ament_package()
