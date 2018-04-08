#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/dumper/move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the dumper/move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the dumper moved to (2, 1)");
    else
        ROS_INFO("The dumper failed to move to (2, 1) for some reason");

    return 0;
}