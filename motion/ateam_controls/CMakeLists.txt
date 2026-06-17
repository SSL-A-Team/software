cmake_minimum_required(VERSION 3.8)
project(ateam_controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(controls)


install(
  DIRECTORY controls/ateam-controls-c/include/
  DESTINATION include
)
install(
  FILES $<TARGET_FILE:ateam_controls> 
  DESTINATION lib
)


_ament_cmake_export_libraries_register_environment_hook()
_ament_cmake_export_libraries_register_package_hook()
list(APPEND _AMENT_EXPORT_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libateam_controls_c.a)
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include)

ament_package()
