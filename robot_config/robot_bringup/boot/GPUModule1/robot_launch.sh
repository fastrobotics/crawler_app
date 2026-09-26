#!/bin/bash
echo "-----BOOTING ROBOT ON GPUModule1 -----"
sleep 10
echo "Cleaning App"
rm -rf /home/robot/var/log/output/*
mkdir -p /home/robot/var/log/output/
touch /home/robot/var/log/output/app_launch.out
sleep 5 # Leave enough time for ROS to start
echo "-----HARDWARE MODIFICATIONS-----"
echo "-----LAUNCHING APPLICATION-----"
cd /home/robot/ros2_ws/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch crawler_app orchestrator.launch.py robot_namespace:=robot > /dev/null 2> /home/robot/var/log/output/app_launch.out &
echo "-----APP LAUNCH FINISHED-----"