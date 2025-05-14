#! /bin/bash

if nmcli radio wifi | grep -q "enabled"; then
  while true; do
    echo -e "\e[31mWIFI IS ENABLED! TURN IT OFF!\e[0m" >&2
    sleep 1
  done
else
  echo "WiFi disabled check passed." >&2
fi
