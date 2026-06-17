#! /usr/bin/env bash

set -e 

if [ -n "$SIMULATORCLI_PATH" ]; then
    echo "ER-Force Simulator already installed."
    exit 0
fi

echo "Installing ER-Force Simulator..."

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

ERFORCE_DIR=$SCRIPT_DIR/../../../../erforce
mkdir $ERFORCE_DIR
pushd $ERFORCE_DIR
git clone https://github.com/robotics-erlangen/framework.git
sudo apt install -y cmake protobuf-compiler libprotobuf-dev qt6-base-dev libqt6opengl6-dev g++ libusb-1.0-0-dev libsdl2-dev libqt6svg6-dev libssl-dev libglu1-mesa-dev
pushd framework
mkdir build
pushd build
cmake ..
cmake --build . --target project_bullet simulator-cli
echo $"export SIMULATORCLI_PATH=$ERFORCE_DIR/framework/build/bin/simulator-cli\n" >> ~/.bashrc
source ~/.bashrc
popd
popd
popd

echo "ER-Force Simulator installed."
