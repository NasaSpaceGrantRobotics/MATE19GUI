#!/bin/bash
gnome-terminal -x "roscore"     #starts roscore
wait

sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/MATE19/driverside/src -e "rmmod xpad"
sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/MATE19/driverside/src -e "xboxdrv --silent"     #starts xboxdriver
wait

gnome-terminal -x bash -c "rosparam set joy_node/dev /dev/input/js2 && rosrun joy joy_node"     #sets joystick to joy node
wait

gnome-terminal -e "roslaunch pid.launch"    #launch pid and input processor
