#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_planning/PickPlaceAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_client");

    actionlib::SimpleActionClient<moveit_planning::PickPlaceAction> ac("pick_place", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();

    moveit_planning::PickPlaceGoal goal;
    goal.object_id = "cube1";
    goal.goal_pose.header.frame_id = "world";
    goal.goal_pose.pose.position.x = 0.5;
    goal.goal_pose.pose.position.y = 0.0;
    goal.goal_pose.pose.position.z = 0.3;

    ac.sendGoal(goal);

    ac.waitForResult();
    ROS_INFO("Result: %s", ac.getResult()->message.c_str());
}
