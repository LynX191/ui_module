#!/bin/bash

sshpass -p "tomo" ssh -t tomo@192.168.0.221 'sudo shutdown -h now'
sshpass -p "tomo" ssh -t tomo@192.168.0.222 'sudo shutdown -h now'
sshpass -p "tomo" ssh -t tomo@192.168.0.223 'sudo shutdown -h now'
sudo shutdown -h now
