# The "Home Service Robot" ROS package

## Project Objective

The purpose of this project is to simulate a "Home Service Robot" that is capable of picking up objects in a known map and dropping them off in other locations. This project uses ROS packages, some from the official ROS repository and others I've written myself in C++.

All portions of this project use the "turtlebot_gazebo" official ROS package in order to load a turtlebot equipped with a kinect sensor into the "douglas.world" Gazebo world. The "turtlebot_gazebo" package also contains a preconfigured rviz config file used throughout the project.

## Mapping with SLAM

The first step after setting up the catkin workspace is to use SLAM to create a map of my Gazebo world. Two additional packages are used to accomplish this: The "turtlebot_teleop" package is used to manually drive the Turtlebot through the world in order to create a map. The second package used is "slam_gmapping" which will create a 2D occupancy grid map of the world as the robot moves around its environment. This map is later used to set navigation goals for the pick-and-place application of the Home Service Robot.

## Navigation & Localization

After creating the map, the next step is to test the localization and navigation of the Home Service Robot. For navigation, I will be using the ROS Navigation Stack, which is based on Dijkstra's Uniform Cost Search Algorithm. The offical ROS Package "turtlebot_gazebo" contains the launch files used for navigation and Adaptive Monte Carlo Localization in this project. The final tests for this project involve setting navigation goals and placing markers to simulate the Home Service Robot picking up an object and dropping it at another location. For these tests I have created my own two packages: "pick_objects" is used to command the robot to navigate to a pick-up zone, pause for 5 seconds, and then move to a drop-off zone. And "add_marker" is used to place markers to represent an object being picked up and dropped off at it's respective zones.

Finally, this project contains a "home_service.sh" script that puts everything together to successfully simulate my Home Service Robot.
