#! /usr/bin/env bash

# Prefer using rosdep for dependencies available via that tool

set -e

echo "Installing APT dependencies..."

sudo apt update
sudo apt upgrade -y

sudo apt install -y python3-clang net-tools

echo "APT dependencies installed."
