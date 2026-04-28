#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# FILE:         bringup_tibble.sh
# AUTHOR:       Ella Moody <moodyellam@gmail.com>
# CREATED:      04-25-2026
# LAST EDITED:  04-28-2026
# DESCRIPTION:  This script brings up ready terminals for onboard and groundstation
#               wireless control. Automates SSH and sudo password entry.
#               Specifically meant to work with Fedora 43 and Ubuntu 24.04. 
#               The -a option will use the wifi antenna instead of the built-in card.
# USAGE:        ./bringup_tibble.sh [-a]
# DEPENDS:      bash, docker, docker compose, docker cli, ssh, tmux, sshpass
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

# Parse optional use antenna argument (default false)
USE_ANTENNA=0
while getopts "a" opt; do
  case ${opt} in
    a ) USE_ANTENNA=1 ;;
    \? ) echo "Usage: ./bringup_tibble.sh [-a]"
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

SESSION_NAME="tibble_bringup"

# Kill the session if it's already running to start fresh
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Create new detached session for groundstation
tmux new-session -d -s $SESSION_NAME

# Enable mouse
tmux set-option -t $SESSION_NAME mouse on

# Groundstation panel setup
# Note: Added 'source ~/.bashrc' to the rcfile payload so default linux commands work
tmux send-keys -t $SESSION_NAME "echo 'Setting up groundstation console...'" C-m
tmux send-keys -t $SESSION_NAME "cd docker" C-m
tmux send-keys -t $SESSION_NAME "docker compose up -d" C-m
tmux send-keys -t $SESSION_NAME "cd .." C-m
tmux send-keys -t $SESSION_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && echo -e \"\n\033[1;32mGroundstation Ready for Launch!\033[0m\n\" && exec bash'" C-m

# Create another panel on the right for onboard
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo 'Setting up onboard console...'" C-m

# Automate sudo commands
SUDO_CMD="echo \"$ONBOARD_SUDO_PASSWORD\" | sudo -S"

# Optionally append the antenna config
if [ "$USE_ANTENNA" = "1" ]; then
    WIFI_CMD="$SUDO_CMD nmcli device disconnect wlp3s0; $SUDO_CMD nmcli device wifi connect \"$WIFI_SSID\" password \"$WIFI_PASSWORD\" ifname $ANTENNA_SERIAL;"
else
    WIFI_CMD="$SUDO_CMD nmcli device wifi connect \"$WIFI_SSID\" password \"$WIFI_PASSWORD\";"
fi

# SSH setup and can bringup
tmux send-keys -t $SESSION_NAME "sshpass -p \"$ONBOARD_SSH_PASSWORD\" ssh -t $ONBOARD_USERNAME@$ONBOARD_IP '
    echo \"Configuring network interfaces...\";
    $WIFI_CMD
    echo \"Bringing up CAN interface (can0)...\";
    $SUDO_CMD ip link set can0 down;
    $SUDO_CMD ip link set can0 up type can bitrate 1000000;
    $SUDO_CMD ip link set can0 txqueuelen 1000;
    $SUDO_CMD ip link set can0 up;
    ip link show can0;
    cd ~/$ONBOARD_REPO_PATH;
    bash --rcfile <(echo \"source ~/.bashrc && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && source /opt/ros/jazzy/setup.bash && source install/setup.bash && echo -e \\\"\n\033[1;32mOnboard ready to launch!\033[0m\n\\\"\");
'" C-m

# Attach current terminal to the newly created session
tmux attach-session -t "$SESSION_NAME"