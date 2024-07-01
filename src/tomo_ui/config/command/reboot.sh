#!/usr/bin/env bash

if zenity --question --text="Are you sure you want to restart all 4 Xaviers?" --width=400; then
    cd /home/tomo/ui_moduleinstall/share/tomo_ui/config/command && sudo bash ./XavierRestart.sh
fi
