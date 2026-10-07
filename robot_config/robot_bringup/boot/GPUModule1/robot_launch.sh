#!/bin/bash

source "$(dirname "${BASH_SOURCE[0]}")/app_launch_helpers.sh"
rm -rf /home/robot/var/log/output/*
mkdir -p /home/robot/var/log/output/
touch /home/robot/var/log/output/app_launch.out
message "-----BOOTING ROBOT ON GPUModule1 -----"
message "Sleeping to wait for Data Logger"
sleep 60
if [ -d "/mnt/usb_storage/datalogs/" ]; then
    message "DataLogger Attached!"
else
    message "DataLogger Disconnected!"
fi

message "-----HARDWARE MODIFICATIONS-----"
export OPENNI2_REDIST=/usr/lib/aarch64-linux-gnu/OpenNI2/Drivers
message "-----LAUNCHING APPLICATION-----"
cd /home/robot/ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch crawler_app orchestrator.launch.py robot_namespace:=robot >> /home/robot/var/log/output/app_launch.out 2>&1 &

message "-----APP LAUNCH FINISHED-----"
