#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# FILE:         bringup_tibble.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      04-25-2026
# LAST EDITED:  04-26-2026
# DESCRIPTION:  This script brings up ready terminals for onboard and groundstation
#               wireless control. It does not launch any ros2, just gets the
#               terminals ready for use on both computers. Specifically meant
#               to work with Fedora 43 and Ubuntu 24.04. The -a option will use
#               the wifi antenna instead of the built in wifi card.
# USAGE:        ./bringup_tibble.sh [-a]
# DEPENDS:      bash, docker, docker compose, docker cli, ssh, sshpasss, tmux
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

# Parse optional use antenna argument (default false)
USE_ANTENNA=0
while getopts "a" opt; do
  case ${opt} in
    a ) USE_ANTENNA=1 ;;
    \? ) echo "Usage: ./bringup.sh [-a]"
         echo "  -a: Configure the optional Wi-Fi antenna on the onboard computer"
         exit 1 ;;
  esac
done

# Load the .env
if [ -f .env ]; then
  set -a
  source .env
  set +a
else
  echo "[ERROR] .env file not found in the current directory."
  exit 1
fi

# Verify sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "[ERROR] sshpass is not installed."
    exit 1
fi

SESSION_NAME="tibble_bringup"

# Kill the session if it's already running to start fresh
tmux kill-session -t $SESSION_NAME 2>/dev/null

# BUILD THE DOCKER CONTAINER TERMINAL
# Check if running -> Start if not -> Exec into it -> Source ROS2 -> Drop to interactive bash
DOCKER_CMD="
if [ \"\$(docker inspect -f '{{.State.Running}}' $DOCKER_CONTAINER_NAME 2>/dev/null)\" != \"true\" ]; then 
    echo 'Starting Docker container...'; 
    docker compose up -d; 
fi; 
docker exec -it $DOCKER_CONTAINER_NAME bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec bash'
"

# BUILD THE ONBOARD TERMINAL
REMOTE_CMDS=""

# Optionally append the antenna config
if [ $USE_ANTENNA -eq 1 ]; then
    REMOTE_CMDS+="echo '$ONBOARD_SUDO_PASSWORD' | sudo -S nmcli device wifi connect '$WIFI_SSID' password '$WIFI_PASSWORD' ifname '$ANTENNA_SERIAL'; "
fi

# Append CAN setup, sourcing, and drop to interactive bash
REMOTE_CMDS+="echo '$ONBOARD_SUDO_PASSWORD' | sudo -S ip link set can0 down; "
REMOTE_CMDS+="echo '$ONBOARD_SUDO_PASSWORD' | sudo -S ip link set can0 up type can bitrate 1000000; "
REMOTE_CMDS+="echo '$ONBOARD_SUDO_PASSWORD' | sudo -S ip link set can0 txqueuelen 1000; "
REMOTE_CMDS+="echo '$ONBOARD_SUDO_PASSWORD' | sudo -S ip link set can0 up; "
REMOTE_CMDS+="cd $ONBOARD_REPO_PATH; "
REMOTE_CMDS+="source /opt/ros/jazzy/setup.bash; "
REMOTE_CMDS+="source install/setup.bash; "
REMOTE_CMDS+="exec bash"

# LAUNCH THE TERMINALS
echo "Launching environment..."

# Pane 0 (Left): Create a new detached tmux session and run the Docker logic
tmux new-session -d -s "$SESSION_NAME" "bash -c \"$DOCKER_CMD\""

# Pane 1 (Right): Split the window horizontally and run the SSH logic
# Note: -o StrictHostKeyChecking=accept-new prevents the script from hanging on first-time connections
tmux split-window -h -t "$SESSION_NAME:0" "sshpass -p '$ONBOARD_SSH_PASSWORD' ssh -o StrictHostKeyChecking=accept-new -t $ONBOARD_USERNAME@$ONBOARD_IP \"$REMOTE_CMDS\""

# Attach your current terminal to the newly created session
tmux attach-session -t "$SESSION_NAME"