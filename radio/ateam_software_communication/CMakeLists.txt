cmake_minimum_required(VERSION 3.8)
project(ateam_software_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(RADIO_PACKETS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/software-communication/ateam-common-packets/include)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${RADIO_PACKETS_INCLUDE_DIR}/
  DESTINATION include/ateam_radio_msgs/ateam_radio_msgs/packets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
