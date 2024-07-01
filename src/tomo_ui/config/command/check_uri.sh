#!/usr/bin/env bash

# Define the IP address to ping
ip_address="192.168.0.224"

# Ping the IP address
if ping -c 1 -W 2 "$ip_address" &> /dev/null; then
    # If ping is successful, execute the restart script and expect script
    expect ~/tomo_config/openUI.exp
else
    # If ping fails, show a message using zenity
    zenity --error --text="Connection issue detected. Please check again!" --width=400 --height=100
fi
