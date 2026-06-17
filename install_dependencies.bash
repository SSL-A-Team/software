#! /usr/bin/env bash

# Run this script to install all dependencies for our software.
# 
# Options:
#  -s: Install simulator dependencies (erforce_simulator)


set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

INSTALL_SIM="false"

while getopts ":s" opt; do
  case $opt in
    s)
      INSTALL_SIM="true"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done

$SCRIPT_DIR/dep_scripts/apt.bash
$SCRIPT_DIR/dep_scripts/ros.bash
$SCRIPT_DIR/dep_scripts/rosdep.bash
$SCRIPT_DIR/dep_scripts/rust.bash
$SCRIPT_DIR/ateam_ui/install_deps.sh

if [ "$INSTALL_SIM" = "true" ]; then
    $SCRIPT_DIR/dep_scripts/erforce_simulator.bash
fi
