cmake_minimum_required(VERSION 3.8)
project(ateam_radio_msgs)

include(cmake/generate_msgs.cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(GENERATED_MSGS_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_msgs)

generate_msgs(
  SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/software-communication/ateam-common-packets/include/radio.h
  DESTINATION ${GENERATED_MSGS_DIR}/msg
  STRUCTS
    BasicTelemetry
    ExtendedTelemetry
    KickerTelemetry
    MotorTelemetry
    PowerTelemetry
    BatteryInfo
)

file(GLOB CONFIG_DEPENDS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/software-communication/ateam-common-packets/include/*.*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CONFIG_DEPENDS})

rosidl_generate_interfaces(${PROJECT_NAME}
  ${GENERATED_MSGS_DIR}:msg/BasicTelemetry.msg
  ${GENERATED_MSGS_DIR}:msg/ExtendedTelemetry.msg
  ${GENERATED_MSGS_DIR}:msg/KickerTelemetry.msg
  ${GENERATED_MSGS_DIR}:msg/MotorTelemetry.msg
  ${GENERATED_MSGS_DIR}:msg/PowerTelemetry.msg
  ${GENERATED_MSGS_DIR}:msg/BatteryInfo.msg

  srv/GetFirmwareParameter.srv
  srv/SetFirmwareParameter.srv

  ADD_LINTER_TESTS
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
