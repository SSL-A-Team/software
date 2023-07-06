#!/bin/bash

sudo ifconfig lo multicast
sudo route add 224.5.23.1 lo
sudo route add 224.5.23.2 lo
