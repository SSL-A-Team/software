#!/usr/bin/env bash

set -e

function add_loopback_route {
  address=$1
  if route | grep -qs "$address"; then
    echo "Route for $address already exists."
  else
    sudo route add "$address" lo
  fi
}

sudo ifconfig lo multicast
add_loopback_route 224.5.23.1  # Game Controller
add_loopback_route 224.5.23.2  # SSL Vision
echo "Multicast should now work on the loopback interface."
