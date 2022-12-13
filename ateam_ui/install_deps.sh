#! /bin/bash

# This script installs javascript dependencies which can't be handled via
# rosdep. It requires internet access. It must be run within the same colcon
# workspace as the ateam_ui package.

set -e

if ! command -v npm; then
  echo "ERROR: npm is not installed"

  echo "attempting to install nodejs and npm"
  sudo apt update -y
  sudo apt install nodejs npm -y
  if ! command -v npm; then
    echo "failed to install npm"
    exit 1
  fi
fi

install_npm_package_if_missing () {
  local package_name=$1
  echo "Checking for npm package: $package_name"
  if npm list --depth 1 --global $package_name > /dev/null; then
    echo "$package_name already installed"
  else
    echo "Installing $package_name"
    sudo npm install -g $package_name &> /dev/null
  fi
}

install_npm_package_if_missing n
echo "Upgrading node to latest stable"
sudo n stable &> /dev/null

install_npm_package_if_missing yarn

install_npm_package_if_missing @neutralinojs/neu

# Assumes the script runs somewhere inside the colcon workspace
ateam_ui_pkg_path=$(colcon --log-base /dev/null list --packages-select ateam_ui -p)

pushd $ateam_ui_pkg_path/src &> /dev/null

echo "Installing UI yarn dependencies"
yarn install

if [ ! -d ./bin ]; then
  echo "Configuring neu"
  neu update
fi

popd &> /dev/null

echo "UI dependencies installed!"
