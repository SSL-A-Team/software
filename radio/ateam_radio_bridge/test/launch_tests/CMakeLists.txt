# TODO(barulicm): These tests are too annoying to maintain when packets update
# Restore these when we switch to protobufs or can auto-generate Python definitions for the packet
# structs automatically.

# find_package(launch_testing_ament_cmake REQUIRED)
# add_launch_test(bridge_discovery_test.py
#   APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
#   TIMEOUT 10
# )
# add_launch_test(bridge_feedback_test.py
#   APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
#   TIMEOUT 10
# )
# add_launch_test(bridge_command_test.py
#   APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
#   TIMEOUT 10
# )
# add_launch_test(bridge_goodbye_test.py
#   APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
#   TIMEOUT 10
# )

# install(PROGRAMS
#   mock_robot.py
#   DESTINATION lib/${PROJECT_NAME}
# )
