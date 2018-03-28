# multi_robot_path_planner
This ROS catkin package implements a node that performs single-robot pathfinding on an obstacle map.

Call `roslaunch multi_robot_path_planner single_bot_stage_nav.launch` to test.

You can call it with a specific map using `roslaunch multi_robot_path_planner single_bot_stage_nav.launch stage_sim:=[launch file in ssr_stage]`