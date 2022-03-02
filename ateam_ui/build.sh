#! /bin/bash

set -e

pushd src > /dev/null

# Build the UI
echo "Building"
yarn vite build
neu build --release

# Fix potential permissions issue
if [ ! -x ./dist/Ateam_UI/neutralino-linux_x64 ]; then
  echo "Fixing executable permissions"
  chmod +x ./dist/Ateam_UI/*
fi

popd > /dev/null

echo "UI Build Complete!"
