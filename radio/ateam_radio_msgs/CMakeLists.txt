cmake_minimum_required(VERSION 3.8)
project(ateam_radio_msgs)

include(cmake/generate_msgs.cmake)
include(cmake/generate_conversion_code.cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/ateam_generated)

set(RADIO_PACKETS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/software-communication/ateam-common-packets/include)

set(RADIO_STRUCTS_TO_GENERATE
    BasicTelemetry
    ExtendedTelemetry
    KickerTelemetry
    MotorTelemetry
    PowerTelemetry
    BatteryInfo
)

generate_msgs(
  SOURCE ${RADIO_PACKETS_INCLUDE_DIR}/radio.h
  DESTINATION ${GENERATED_DIR}/msg
  STRUCTS ${RADIO_STRUCTS_TO_GENERATE}
)

generate_conversion_code(
  SOURCE ${RADIO_PACKETS_INCLUDE_DIR}/radio.h
  DESTINATION ${GENERATED_DIR}
  STRUCTS ${RADIO_STRUCTS_TO_GENERATE}
)

file(GLOB CONFIG_DEPENDS CONFIGURE_DEPENDS "${RADIO_PACKETS_INCLUDE_DIR}/*.*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CONFIG_DEPENDS})

set(MSG_TARGET ${PROJECT_NAME})
rosidl_generate_interfaces(${MSG_TARGET}
  ${GENERATED_DIR}:msg/BasicTelemetry.msg
  ${GENERATED_DIR}:msg/ExtendedTelemetry.msg
  ${GENERATED_DIR}:msg/KickerTelemetry.msg
  ${GENERATED_DIR}:msg/MotorTelemetry.msg
  ${GENERATED_DIR}:msg/PowerTelemetry.msg
  ${GENERATED_DIR}:msg/BatteryInfo.msg

  msg/ConnectionStatus.msg

  srv/GetFirmwareParameter.srv
  srv/SetFirmwareParameter.srv

  ADD_LINTER_TESTS
)

set(LIB_TARGET ${PROJECT_NAME}_lib)
add_library(${LIB_TARGET} SHARED
  ${GENERATED_DIR}/src/conversion.cpp
)
target_include_directories(${LIB_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GENERATED_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(${LIB_TARGET} PROPERTIES CXX_STANDARD 20)
rosidl_get_typesupport_target(cpp_typesupport_target ${MSG_TARGET} rosidl_typesupport_cpp)
target_link_libraries(${LIB_TARGET} PUBLIC ${cpp_typesupport_target})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
  PATTERN packets EXCLUDE
)
install(
  DIRECTORY ${GENERATED_DIR}/include/
  DESTINATION include/ateam_radio_msgs/ateam_radio_msgs
)
file(GLOB SYMLINKED_INCLUDES include/ateam_radio_msgs/packets/*)
install(
  FILES ${SYMLINKED_INCLUDES}
  DESTINATION include/ateam_radio_msgs/ateam_radio_msgs/packets
)
install(
  TARGETS ${LIB_TARGET}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  file(GLOB_RECURSE submodule_files software-communication/*)
  file(GLOB symlinked_headers include/ateam_radio_msgs/packets/*)
  set(_linter_excludes
    software-communication
    ${submodule_files}
    ${symlinked_headers}
  )

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(EXCLUDE ${_linter_excludes})

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(EXCLUDE ${_linter_excludes})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
