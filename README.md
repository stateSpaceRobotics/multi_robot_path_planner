# multi_robot_path_planner
This ROS catkin package implements a node that performs single- or multi-robot pathfinding on an obstacle map.

## params.yaml
Here is where all of the configuration is done.  You can specify whether or not to use rviz, what map provider to use, what environment (stage, gazebo, etc) to launch, and specify a list of robots of arbitrary size.  Look at the file to see the structure.  For now, these parameters must be loaded into the global namespace, but they are encapsulated in /navigation

## navigation.launch
Run this launch file to upload the parameters to the parameter server and start the navigation node.

## launch_nav.py
This file will read in the parameters from the parameter server and launch all of the required launch files with their given parameters.

## Notes
The reason this is in feature/ros-lunar-parameterization is because, contrary to what the wiki currently shows, passing arguments through the roslaunch python api is first supported in lunar, not kinetic.  If desired, we can patch the required ros file (5-6 line edit; tested it on my machine and it works flawlessly) and run this on kinetic.

## Resources
[ROS Navigation Wiki](http://wiki.ros.org/navigation/Tutorials/RobotSetup) - Only has instructions for single bot.

[Multi Robot Package](https://github.com/gergia/multiple_turtlebots_stage_amcl/tree/master) - Works for 2 robots as far as I can tell.  This is what I am basing our implementation off of.  Does not solve issue of bots without sensors.

[ROS Navigation_stage Package](http://wiki.ros.org/navigation_stage) - Has a multi-robot launch file, but it does not seem to work correctly.  Using the .rviz files (with edits) from here for visualization.

[Forum Answer](https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/) - Instructions on setting up a multi-robot system (uses gazebo).

[ROS Distributed Mapping](http://wiki.ros.org/nav2d/Tutorials/DistributedMapping) - May help when we try to get SLAM working in a multi-robot system.  We will not be mapping with multiple robots, but this could be helpful.