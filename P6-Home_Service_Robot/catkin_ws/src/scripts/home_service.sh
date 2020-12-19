#!/bin/sh
#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/world/douglas.world" &
sleep 5
#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find turtlebot_gazebo)/maps/playground.yaml" &
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml initial_pose_a:=-1.57" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e   " rosrun add_markers add_markers" &
sleep 5
xterm -e   " rosrun pick_objects pick_objects"