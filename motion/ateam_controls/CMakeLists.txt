cmake_minimum_required(VERSION 3.8)
project(ateam_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/controls/target)

if((CMAKE_BUILD_TYPE STREQUAL "Release") OR (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/release/libateam_controls_c.a)
else()
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/debug/libateam_controls_c.a)
endif()

add_subdirectory(controls)


install(
  DIRECTORY controls/ateam-controls-c/include/
  DESTINATION include
)
install(
  FILES $<TARGET_FILE:ateam_controls> 
  DESTINATION lib
)

ament_export_libraries(-l${CMAKE_INSTALL_PREFIX}/lib/libateam_controls_c.a)
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include)

ament_package()
