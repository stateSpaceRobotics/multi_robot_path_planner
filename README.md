# multi_robot_path_planner
This ROS catkin package implements a node that performs single- or multi-robot pathfinding on an obstacle map.

## single robot
 - single_bot_stage.launch
    - launches navigation stack on a single robot with map_server providing a map, amcl providing localization, and rviz providing visuals and control via the ability to publish nav goals
    - optional argument: stage_sim:=<launch file in ssr_stage>
    - optional argument: map_file:=<map .yaml file in ssr_stage>
    - make sure that the map file is the one corresponding to the launch file you are using

 - single_bot_stage_gmapping.launch
    - launches navigation stack on a single robot with gmapping providing a map and localization, and rviz providing visuals and control via the ability to publish nav goals
    - optional argument: stage_sim:=<launch file in ssr_stage>

## multi robot
 - multi_bot_stage.launch
    - launches navigation stack on multiple robots with map_server providing a map, amcl providing localization, and rviz providing visuals
    - optional argument: stage_sim:=<launch file in ssr_stage>
    - optional argument: map_file:=<map .yaml file in ssr_stage>
    - make sure that the map file is the one corresponding to the launch file you are using

## Resources
[ROS Navigation Wiki](http://wiki.ros.org/navigation/Tutorials/RobotSetup) - Only has instructions for single bot.

[Multi Robot Package](https://github.com/gergia/multiple_turtlebots_stage_amcl/tree/master) - Works for 2 robots as far as I can tell.  This is what I am basing our implementation off of.  Does not solve issue of bots without sensors.

[ROS Navigation_stage Package](http://wiki.ros.org/navigation_stage) - Has a multi-robot launch file, but it does not seem to work correctly.  Using the .rviz files (with edits) from here for visualization.

[Forum Answer](https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/) - Instructions on setting up a multi-robot system (uses gazebo).

[ROS Distributed Mapping](http://wiki.ros.org/nav2d/Tutorials/DistributedMapping) - May help when we try to get SLAM working in a multi-robot system.  We will not be mapping with multiple robots, but this could be helpful.