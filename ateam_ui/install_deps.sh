#! /bin/bash

# This script installs javascript dependencies which can't be handled via
# rosdep. It requires internet access. It must be run within the same colcon
# workspace as the ateam_ui package.

set -e

install_npm_package_if_missing () {
  local package_name=$1
  echo "Checking for npm package: $package_name"
  if npm list --depth 1 --global $package_name > /dev/null 2>&1; then
    echo "$package_name already installed"
  else
    echo "Installing $package_name"
    sudo npm install -g $package_name &> /dev/null
  fi
}

echo "Installing libwebkit2gtk"
# libwebkit2gtk is usually installed by default on desktop Ubuntu, but is
# missing in some environments (including our CI)
sudo apt-get update
sudo apt-get install -y libwebkit2gtk-4.0-37

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
