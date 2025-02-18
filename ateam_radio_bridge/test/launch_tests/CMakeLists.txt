find_package(launch_testing_ament_cmake REQUIRED)
add_launch_test(bridge_discovery_test.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  TIMEOUT 10
)
add_launch_test(bridge_feedback_test.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  TIMEOUT 10
)
add_launch_test(bridge_command_test.py
  APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  TIMEOUT 10
)

install(PROGRAMS
  mock_robot.py
  DESTINATION lib/${PROJECT_NAME}
)
