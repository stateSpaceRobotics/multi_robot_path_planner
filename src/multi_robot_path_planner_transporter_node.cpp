#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/transporter/move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the transporter/move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    float y = 1.5;
    while (true)
    {
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 2.0;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, the transporter moved to its goal.");
        }
        else
        {
            ROS_INFO("The transporter failed to move to its goal for some reason");
        }

        if (y == 1.5)
        {
            y = 5.0;
        }
        else
        {
            y = 1.5;
        }
    }
    return 0;
}