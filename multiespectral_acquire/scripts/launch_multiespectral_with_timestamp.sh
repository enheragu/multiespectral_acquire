#!/bin/bash
# Helper script to launch multiespectral acquisition with automatic timestamp

SESSION_FOLDER=$(date +'%d-%m-%Y_%Hh%Mm')

echo "Launching multiespectral acquisition system..."
echo "Session folder: $SESSION_FOLDER"
echo ""

# Optional: Configure GigE network for FLIR camera (reduces "image incomplete" errors)
# Uncomment the next line if you experience frequent image incomplete errors
# $(dirname "$0")/setup_gige_network.sh

roslaunch multiespectral_acquire multiespectral_launch.launch \
    session_folder:="$SESSION_FOLDER" \
    "$@"
