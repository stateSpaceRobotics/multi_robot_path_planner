# multi_robot_path_planner
This ROS catkin package implements a node that performs single- or multi-robot pathfinding on an obstacle map. It is currently a work in progress.

### Installing dependencies
To install dependencies, cd to your catkin_ws folder and run the following command:
```sh
rosdep install --from-paths ./ --ignore-src --rosdistro=kinetic
```

### Architecture of Project (along with 2019_robot)
The following two architecture diagrams show which packages launch which other packages and what they do. In the current architecture, multi_robot_path_planner provides many things that should be handled by 2019_robot. The proposed architecture moves these to 2019_robot in order to be more modular and focused on only navigation in this package.

Current Architecture:
```
2019_robot
├── multi_robot_path_planner (provides robot configurations)
│   ├── map servers or gmapping (provides map or package to build map)
│   ├── stage simulation (simulates real robots and obstacles)
│   ├── move_base, move_base_recovery (provides server to send movement commands to)
│   ├── amcl or fake localization (provides details on robots' locations)
│   └── rviz (visualization of maps, robots, sensors, transforms, paths, etc)
├── state machine (provides logic that controls the robots' actions)
└── smach_viewer (visualization of the state machine)
```

Proposed Architecture:
```
2019_robot (provides robot configurations)
├── stage simulation or real robot initialization
├── rviz
├── multi_robot_path_planner
│   ├── map servers or gmapping
│   ├── move_base, move_base_recovery
│   ├── amcl or fake localization
├── state machine
└── smach_viewer
```

### Folders and Files in this Repository

```
multi_robot_path_planner
├── CMakeLists.txt
├── launch
│   ├── multi_bot_stage.launch
│   ├── single_bot_stage_gmapping.launch
│   ├── single_bot_stage.launch
│   └── single_robot.launch
├── move_base_config
│   ├── amcl_node.xml
│   ├── fake_localization_node.xml
│   ├── slam_gmapping.xml
│   └── stage
│       ├── digger
│       │   ├── base_local_planner_params.yaml
│       │   ├── costmap_common_params.yaml
│       │   ├── global_costmap_params.yaml
│       │   └── local_costmap_params.yaml
│       ├── dumper
│       │   ├── base_local_planner_params.yaml
│       │   ├── costmap_common_params.yaml
│       │   ├── global_costmap_params.yaml
│       │   └── local_costmap_params.yaml
│       └── transporter
│           ├── base_local_planner_params.yaml
│           ├── costmap_common_params.yaml
│           ├── global_costmap_params.yaml
│           └── local_costmap_params.yaml
├── multi_robot.rviz
├── package.xml
├── README.md
├── scripts
│   └── Recovery_State_Machine.py
├── single_robot.rviz
└── src
    ├── multi_robot_path_planner_digger_node.cpp
    ├── multi_robot_path_planner_dumper_node.cpp
    └── multi_robot_path_planner_transporter_node.cpp
```

 - CMakeLists.txt: This file controls how catkin builds the package to be used by ROS. Editted rarely. Details in the [wiki](http://wiki.ros.org/catkin/CMakeLists.txt).
 - launch: This folder holds our launch files. Details on launch file syntax and usage found in the [wiki](http://wiki.ros.org/roslaunch?distro=kinetic).
    - multi_bot_stage.launch: Launches map servers, stage and rviz, as well as the navigation stack for each robot using single_robot.launch.
    - single_bot_stage_gmapping.launch: Launches gmapping, stage and rviz, as well as the navigation stack for a single robot using single_robot.launch.
    - single_bot_stage.launch: Launches map server, stage and rviz, as well as the navigation stack for each robot using single_robot.launch.
    - single_robot.launch: Launches move_base and move_base_recovery for a robot, as well as either amcl or fake localization for localizing the robot.
 - move_base_config: This folder holds the configurations needed for this package.
    - amcl_node.xml: Launches the amcl package with lots of parameters set. AMCL is used for robot localization. Can be used in a single-robot situation or multi-robot situation, specified by the arguments passed to it. Details on amcl can be found in the [wiki](http://wiki.ros.org/amcl?distro=kinetic).
    - fake_localization_node.xml: Launches the fake localization package with parameters set. Fake localization pretends that we know exactly where we are and is good for initial testing. Details can be found in the [wiki](http://wiki.ros.org/fake_localization?distro=kinetic).
    - slam_gmapping.xml: Launches the gmapping package with lots of parameters set. This allows us to create maps from sensor readings instead of being given maps. Useful for creating maps to be used in simulations as well. Details can be found in the [wiki](http://wiki.ros.org/gmapping?distro=kinetic).
    - stage: This folder holds robot configurations to be used in stage simulations.
        - digger/dumper/transporter: These folders contain configuration files for the three robots.
            - base_local_planner_params.yaml: This file contains configuration for three different planners, TrajectoryPlannerRos ([wiki](http://wiki.ros.org/base_local_planner?distro=kinetic)), DWAPlannerRos ([wiki](http://wiki.ros.org/dwa_local_planner?distro=kinetic)), and Global_Planner ([wiki](http://wiki.ros.org/global_planner?distro=kinetic)). Global Planner generates a global plan for a robot to follow to a goal, while TrajectoryPlannerRos and DWAPlannerRos generate local plans that attempt to follow the global plan.
            - costmap_common_params.yaml: This file contains configurations that should be used for both the global and local costmap. Details can be found in the [wiki](http://wiki.ros.org/costmap_2d?distro=kinetic).
            - global_costmap_params.yaml: This file contains configurations that should only be used for the global costmap.
            - local_costmap_params.yaml: This file contains configurations that should only be used for the local costmap.
 - multi_robot.rviz: A set of settings for rviz to use when navigating with multiple robots.
 - package.xml: This file specifies the name and version of our package along with what dependencies it has. More details can be found on the [wiki](http://wiki.ros.org/catkin/package.xml).
 - scripts: This folder contains any scripts that we write.
    - Recovery_State_Machine.py: This creates a state machine that generates a goal close to the robot to use as a recovery behavior. If a robot gets stuck, it will attempt to move to this new closer goal and then resume navigation. This can repeat many times before becoming unstuck. In its current implementation, this is called from the 2019_robot state machine.
 - single_robot.rviz: A set of settins for rviz to use when navigating with a single robot.
 - src: Source files for nodes.
    - multi_robot_path_planner_(digger/dumper/transporter)_node.cpp: These nodes send commands to move the corresponding robot. This is only used for testing that movement works.

### Launching this package

##### single robot
 - single_bot_stage.launch
    - launches navigation stack on a single robot with map_server providing a map, amcl providing localization, and rviz providing visuals and control via the ability to publish nav goals
    - optional argument: stage_sim:=<launch file in ssr_stage>
    - optional argument: map_file:=<map .yaml file in ssr_stage>
    - make sure that the map file is the one corresponding to the launch file you are using

 - single_bot_stage_gmapping.launch
    - launches navigation stack on a single robot with gmapping providing a map and localization, and rviz providing visuals and control via the ability to publish nav goals
    - optional argument: stage_sim:=<launch file in ssr_stage>

##### multi robot
 - multi_bot_stage.launch
    - launches navigation stack on multiple robots with map_server providing a map, amcl providing localization, and rviz providing visuals
    - optional argument: stage_sim:=<launch file in ssr_stage>
    - optional argument: map_file:=<map .yaml file in ssr_stage>
    - make sure that the map file is the one corresponding to the launch file you are using

### Resources
[ROS Navigation Wiki](http://wiki.ros.org/navigation/Tutorials/RobotSetup) - Only has instructions for single bot.

[Multi Robot Package](https://github.com/gergia/multiple_turtlebots_stage_amcl/tree/master) - Works for 2 robots as far as I can tell.  This is what I am basing our implementation off of.  Does not solve issue of bots without sensors.

[ROS Navigation_stage Package](http://wiki.ros.org/navigation_stage) - Has a multi-robot launch file, but it does not seem to work correctly.  Using the .rviz files (with edits) from here for visualization.

[Forum Answer](https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/) - Instructions on setting up a multi-robot system (uses gazebo).

[ROS Distributed Mapping](http://wiki.ros.org/nav2d/Tutorials/DistributedMapping) - May help when we try to get SLAM working in a multi-robot system.  We will not be mapping with multiple robots, but this could be helpful.
