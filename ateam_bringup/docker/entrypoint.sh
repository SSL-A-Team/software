#!/bin/bash

# dont set -e since this could be used as a dev env

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "${WORKSPACE}/install/setup.sh"
exec "$@"