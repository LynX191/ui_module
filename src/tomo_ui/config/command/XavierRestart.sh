#!/bin/bash

ssh -t tomo@192.168.0.221 'sudo reboot'
sshpass -p "tomo" ssh -t tomo@192.168.0.222 'sudo reboot'
sshpass -p "tomo" ssh -t tomo@192.168.0.223 'sudo reboot'
sudo reboot
