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
# USAGE:        sudo ./bringup_tibble.sh [-a]
# DEPENDS:      bash, sudo, docker, docker compose, docker cli, ssh
# LICENSE:      Apache 2.0
# -----------------------------------------------------------------------------

set -euo pipefail

# Parse arguments
USE_ANTENNA="false"
while getopts "a" opt; do
    case ${opt} in
        a ) USE_ANTENNA="true" ;;
        \? ) echo "Usage: $0 [-a]" >&2; exit 1 ;;
    esac
done
shift $((OPTIND -1))

#Load .env file for variables
ENV_FILE="$(dirname "$0")/.env"

if [ -f "$ENV_FILE" ]; then
    # Automatically export all variables loaded from the .env file
    set -a
    source "$ENV_FILE"
    set +a
    echo "Configuration loaded from $ENV_FILE"
else
    echo "ERROR: Configuration file not found at $ENV_FILE"
    echo "Please create a .env file with your required variables."
    exit 1
fi

echo "Initiating Tibble Bringup Sequence..."

# Check if Fedora or Ubuntu 24.04
if command -v konsole >/dev/null 2>&1; then
    LAUNCH_CMD="konsole --new-tab -e bash -c"
elif command -v gnome-terminal >/dev/null 2>&1; then
    LAUNCH_CMD="gnome-terminal --tab -- bash -c"
else
    echo "ERROR: No supported terminal emulator found (Konsole or GNOME Terminal)."
    exit 1
fi

# Set up the groundstation console
$LAUNCH_CMD "
    echo 'Setting up groundstation console...';
    cd $REPO_PATH_GROUNDSTATION;

    docker compose up -d;
    
    echo 'Attaching to container...';
    docker exec -it $DOCKER_CONTAINER_NAME bash --rcfile <(echo '. /opt/ros/jazzy/setup.bash && . install/setup.bash && echo -e \"\n\033[1;32mGroundstation Ready for Launch!\033[0m\n\"');
" &


# Set up onboard console
$LAUNCH_CMD "
    echo 'Setting up onboard console...';
    
    ssh -t $MINI_PC_USER@$MINI_PC_IP '
        echo \"Configuring network interfaces...\";
        
        if [ \"$USE_ANTENNA\" = \"true\" ]; then
            echo \"Antenna flag (-a) detected. Using external antenna...\";
            sudo nmcli device disconnect wlp3s0;
            sudo nmcli device wifi connect \"$WIFI_SSID\" password \"$WIFI_PASS\" ifname $ANTENNA_SERIAL;
        else
            echo \"No antenna flag detected. Using built-in wifi...\";
            sudo nmcli device wifi connect \"$WIFI_SSID\" password \"$WIFI_PASS\";
        fi

        echo \"Bringing up CAN interface (can0)...\";
        sudo ip link set can0 down;
        sudo ip link set can0 up type can bitrate 1000000;
        sudo ip link set can0 txqueuelen 1000;
        sudo ip link set can0 up;
        
        echo \"Checking CAN status...\";
        ip link show can0;

        echo \"Entering workspace...\";
        cd ~/GitHub/2026-tibble-bot/;
        
        bash --rcfile <(echo '. install/setup.bash && echo -e \"\n\033[1;32mOnboard Ready for Launch!\033[0m\n\"');
    '
" &

echo "SUCCESS. Both terminals should be open and ready for groundstation and onboard commands."