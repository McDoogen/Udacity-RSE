#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 15
xterm -e   " rosrun pick_objects pick_objects"

#you leave both the turtlebot_world.launch and amcl_demo.launch in their original state
#You simply do `roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<file_path>`
#and do `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<file_path>`
#Now how you make the filepath relative

#`world_file:= $(rospack find wall_follower)/../worlds/<filename>` (Or some other package in your src folder you know the position off, and then set the relative position of your file relative to that #package. This is BEST)
#`world_file:= $(pwd)/../World/<filename>` , this just takes the current directory of script execution and navigates to the World Directory
#Here is the documentation for the rospack find command
