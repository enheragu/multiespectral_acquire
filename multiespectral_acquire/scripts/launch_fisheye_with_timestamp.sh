#!/bin/bash
# Helper script to launch fisheye acquisition with automatic timestamp

SESSION_FOLDER=$(date +'%d-%m-%Y_%Hh%Mm')

echo "Launching fisheye acquisition system..."
echo "Session folder: $SESSION_FOLDER"
echo ""

roslaunch multiespectral_acquire fisheye_launch.launch \
    session_folder:="$SESSION_FOLDER" \
    "$@" --wait
