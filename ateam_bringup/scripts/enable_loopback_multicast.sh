#!/usr/bin/env bash

sudo ifconfig lo multicast
sudo route add 224.5.23.1 lo
sudo route add 224.5.23.2 lo
echo "Multicast should now work on the loopback interface."
