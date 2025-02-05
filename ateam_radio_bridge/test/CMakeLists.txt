add_subdirectory(launch_tests)
add_subdirectory(unit_tests)

install(PROGRAMS
  mock_robot.py
  DESTINATION lib/${PROJECT_NAME}
)
