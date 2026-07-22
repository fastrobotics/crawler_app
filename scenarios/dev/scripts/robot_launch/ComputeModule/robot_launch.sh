#!/bin/bash
echo "-----BOOTING ROBOT-----"
sleep 10
echo "Cleaning App"
rm -rf /home/robot/var/log/output/*
source /opt/ros/noetic/setup.bash
echo "Starting roscore"
roscore &
echo "roscore Started."
sleep 5 # Leave enough time for ROS to start
echo "-----LAUNCHING APPLICATION-----"
cd /home/robot/catkin_ws/
source devel/setup.bash
export ROS_PYTHON_LOG_TO_STDOUT=1
export ROS_HOSTNAME=$(hostname)
export ROS_MASTER_URI=http://$(hostname):11311
roslaunch crawler_app SystemLaunch.launch > /dev/null 2> /home/robot/var/log/output/app_launch.out &
echo "-----APP LAUNCH FINISHED-----"