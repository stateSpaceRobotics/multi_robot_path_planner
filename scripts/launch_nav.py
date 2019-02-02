#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import String


def navigation_node():
    rospy.init_node('navigation')

    # NAV_NAMESPACE = '/navigation'
    ROBOT_NAMESPACE = '/robots'
    PACKAGE_NAME = 'multi_robot_path_planner'
    cli_args = []

    # Get params from parameter server
    # navigation = rospy.get_param(NAV_NAMESPACE)
    robots = rospy.get_param(ROBOT_NAMESPACE)

    # Extract navigation params
    # mapping = navigation.get('mapping', {})

    # Set up mapping launch args for either map_server or gmapping (for now)
    # for mapping_node in mapping:
    #     if mapping_node['node_package'] == 'map_server':
    #         cli_args.append([PACKAGE_NAME, 'map_server.launch',
    #                          'map_package:=' + mapping_node['map_package'],
    #                          'map_path:=' + mapping_node['map_path'],
    #                          'map_frame:=' + mapping_node['map_frame'],
    #                          'map_topic:=' + mapping_node['map_topic'],
    #                          'node_name:=' + mapping_node['node_name']])

    #     elif mapping_node['node_package'] == 'gmapping':
    #         cli_args.append([PACKAGE_NAME, 'gmapping.launch',
    #                          'map_frame:=' + mapping_node['map_frame'],
    #                          'map_topic:=' + mapping_node['map_topic'],
    #                          'odom_frame:=' + mapping_node['odom_frame'],
    #                          'scan_topic:=' + mapping_node['scan_topic'],
    #                          'node_name:=' + mapping_node['node_name']])

    # Set up robot launch args
    for robot in robots:
        cli_args.append([PACKAGE_NAME, 'single_robot.launch'])
        tf_prefix = robots[robot]['tf_prefix']

        for arg, value in robots[robot].iteritems():
            if arg == 'move_base':
                cli_args[-1].append('move_base_config_package:=' +
                                    value['config_package'])
                cli_args[-1].append('move_base_config_path:=' +
                                    value['config_path'])

            elif arg == 'localization':
                cli_args[-1].append('localization_package:=' +
                                    value['package'])
                if value['footprint_frame'].startswith('/'):
                    cli_args[-1].append('localization_footprint_frame:=' +
                                        value['footprint_frame'])
                elif tf_prefix == '' or tf_prefix.endswith('/'):
                    cli_args[-1].append('localization_footprint_frame:=' +
                                        tf_prefix + value['footprint_frame'])
                else:
                    cli_args[-1].append('localization_footprint_frame:=' +
                                        tf_prefix + '/' + value['footprint_frame'])
            elif arg.endswith('frame'):
                if value.startswith('/'):
                    cli_args[-1].append(arg + ':=' + value)
                elif tf_prefix == '' or tf_prefix.endswith('/'):
                    cli_args[-1].append(arg + ':=' +
                                        tf_prefix + value)
                else:
                    cli_args[-1].append(arg + ':=' +
                                        tf_prefix + '/' + value)

            else:
                cli_args[-1].append(arg + ':=' + value)

    # Resolve launch file names
    launch_files = []
    for i in range(len(cli_args)):
        for j in range(len(cli_args[i])):
            print cli_args[i][j]
        print '--------------------'
        launch_files.append(
            (roslaunch.rlutil.resolve_launch_arguments(cli_args[i])[0], cli_args[i][2:]))

    # Create parent and launch all files
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    parent.start()

    rospy.spin()

    parent.shutdown()


if __name__ == '__main__':
    try:
        navigation_node()
    except rospy.ROSInterruptException:
        pass
