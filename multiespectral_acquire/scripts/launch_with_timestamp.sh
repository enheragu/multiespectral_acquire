#!/bin/bash
# Helper script to launch multiespectral acquisition with automatic timestamp

SESSION_FOLDER=$(date +'%d-%m-%Y_%Hh%Mm')

echo "Launching multiespectral acquisition system..."
echo "Session folder: $SESSION_FOLDER"
echo ""

roslaunch multiespectral_acquire multiespectral_launch.launch \
    session_folder:="$SESSION_FOLDER" \
    "$@"
