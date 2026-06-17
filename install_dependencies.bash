#! /usr/bin/env bash

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

$SCRIPT_DIR/dep_scripts/apt.bash
$SCRIPT_DIR/dep_scripts/ros.bash
$SCRIPT_DIR/dep_scripts/rosdep.bash
$SCRIPT_DIR/dep_scripts/rust.bash
$SCRIPT_DIR/ateam_ui/install_deps.sh
$SCRIPT_DIR/dep_scripts/erforce_simulator.bash
