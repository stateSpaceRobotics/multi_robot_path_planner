# multi_robot_path_planner
This ROS catkin package implements a node that performs single-robot pathfinding on an obstacle map.

Call `roslaunch multi_robot_path_planner single_bot_stage_nav.launch` to test.

You can call it with a specific map using `roslaunch multi_robot_path_planner single_bot_stage_nav.launch stage_sim:=[launch file in ssr_stage]`
## Currently issue:
Creating the TF tree with multiple robots.  Map needs to be the root frame, with the robot odom frames each connecting to it.

## Resources
[ROS Navigation Wiki](http://wiki.ros.org/navigation/Tutorials/RobotSetup) - This has instructions on setting it up with a single bot.
[Forum Answer](https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/) - Instructions on setting up a multi-robot system (uses gazebo).
[ROS Navigation_stage Package](http://wiki.ros.org/navigation_stage) - Has a multi-robot launch file, but it does not seem to work correctly.
[Multi Robot Package](https://github.com/gergia/multiple_turtlebots_stage_amcl/tree/master) - This seems to work when you launch it (maybe not the TF frames). Need to look more into it.
[ROS Distributed Mapping](http://wiki.ros.org/nav2d/Tutorials/DistributedMapping) - Haven't looked into it yet, but may have something we can use.