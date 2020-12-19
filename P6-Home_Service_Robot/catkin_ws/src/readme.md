The following packages were used to simulate the "Home Service Robot"

- add_markers: The add_markers package manages the location of the marker simulating an object being picked up and dropped off by the robot.
- pick_objects: The pick_objects package sends poses to the robot to navigate in the world where the marker locations are. 
- my_robot: The my_robot package contains my gazebo world file and map files.
- slam_gmapping: The slam_gmapping package was used to generate a map file
- turtlebot_gazebo: The turtlebot_gazebo package was used to launch the turtlebot inside of the world I made. It also contained the initial rviz configuration as well as the amcl node used for localization.