#! /usr/bin/env bash

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

source /opt/ros/jazzy/setup.bash
pushd $SCRIPT_DIR/..
rosdep install --from-paths . --ignore-src -y
popd
