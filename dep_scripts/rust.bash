#! /usr/bin/env bash

set -e

if command -v rustc &> /dev/null; then
    echo "Rust already installed."
    exit 0
fi

echo "Installing Rust..."

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

echo "Rust installed."
