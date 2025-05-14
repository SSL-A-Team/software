#! /bin/bash

while nmcli radio wifi | grep -q "enabled"; do
  echo -e "\e[31mWIFI IS ENABLED! TURN IT OFF!\e[0m" >&2
  sleep 1
done
