#!/usr/bin/env python

import rospy

from smach import StateMachine
from smach_ros import SimpleActionState, ActionServerWrapper
from move_base_msgs.msg import *
from actionlib import SimpleActionClient
from random import uniform
from math import pi, sin, cos


def create_machine(topic, base_frame):
    # Construct state machine
    sm = StateMachine(outcomes=['succeeded',
                                'aborted',
                                'preempted'],
                      input_keys=['goal_message'])

    with sm:
        StateMachine.add('move_base',
                         SimpleActionState(topic, MoveBaseAction,
                                           goal_key='goal_message'),
                         transitions={'succeeded': 'succeeded',
                                      'aborted': 'close_goal',
                                      'preempted': 'move_base'},
                         remapping={'goal_message': 'goal_message'})

        def close_goal_cb(userdata, goal):

            distance = uniform(0.01, 0.15)
            angle = uniform(0, 2 * pi)
            x, y = distance * cos(angle), distance * sin(angle)

            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = base_frame
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = x
            drive_goal.target_pose.pose.position.y = y
            drive_goal.target_pose.pose.orientation.w = 1.0

            return drive_goal

        StateMachine.add('close_goal',
                         SimpleActionState(topic, MoveBaseAction,
                                           goal_cb=close_goal_cb),
                         transitions={'succeeded': 'move_base',
                                      'aborted': 'close_goal',
                                      'preempted': 'close_goal'})

    return sm


if __name__ == '__main__':
    rospy.init_node('move_base_recovery')

    topic = rospy.get_param('~topic')
    base_frame = rospy.get_param('~base_frame')

    move_base = SimpleActionClient(topic, MoveBaseAction)

    move_base.wait_for_server()

    sm_recovery = create_machine(topic, base_frame)

    recovery_server = ActionServerWrapper(
        topic + '_recovery', MoveBaseAction, sm_recovery,
        ['succeeded'], ['aborted'], ['preempted'],
        goal_key='goal_message')

    recovery_server.run_server()

    rospy.spin()
