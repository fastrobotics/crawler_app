boot_log=~/var/log/output/boot_log.txt
rm ~/var/log/output/*
touch $boot_log
echo "User Boot Started at "$(date) on $(hostname) >> $boot_log
sleep 5
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
source devel/setup.bash
export ROS_HOSTNAME=$(hostname)
export ROS_MASTER_URI=http://$(hostname):11311
sleep 1
users >> $boot_log
env | grep ROS >> $boot_log
echo $ROS_MASTER_URI >> $boot_log
sleep 1
screen -dmS roscore_launch -L bash -c 'roscore; exec bash'
sleep 5
roslaunch config SystemLaunch.launch
echo "User Boot Finished at "$(date) >> $boot_log