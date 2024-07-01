#!/bin/bash

LOG_DIR=~/tomo_stats/ui_log
MAX_LOG_SIZE_KB=10000  # 10 MB

# Ensure the directory exists or create it
mkdir -p "$LOG_DIR"

# Initialize LOG_FILE
LOG_FILE="$LOG_DIR/ui_$(date +%Y_%m_%d-%H_%M_%S).htm"

# Function to check log size and create a new file if needed
check_log_size() {
    if [ -e "$LOG_FILE" ]; then
        local current_size=$(du -k "$LOG_FILE" | cut -f1)
        if [ "$current_size" -ge "$MAX_LOG_SIZE_KB" ]; then
            LOG_FILE="$LOG_DIR/ui_$(date +%Y_%m_%d-%H_%M_%S).htm"
        fi
    fi
}

# Launch TomO-UI and redirect output to both terminal and file
sudo bash -c "cd ~/ws_ui && source ~/ui_moduleinstall/setup.bash && unbuffer roslaunch tomo_ui tomo_ui.launch" 2>&1 | \
    while read -r line; do
        check_log_size
        echo "$line" | stdbuf -oL ansi2html >> "$LOG_FILE"
        echo "$line"
    done




