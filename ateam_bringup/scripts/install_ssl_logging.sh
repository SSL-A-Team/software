#!/usr/bin/env bash

set -e

sudo apt install -y golang

if cat $HOME/.bashrc | grep -qs "$GOROOT"; then
    echo "go root already set"
else
    echo "setting new GOROOT"
    echo 'export GOROOT=$HOME/go
export GOPATH=$HOME/go
export PATH=$PATH:$GOROOT/bin:$GOPATH/bin'>> $HOME/.bashrc
    source $HOME/.bashrc
fi

go install github.com/RoboCup-SSL/ssl-go-tools/...@latest