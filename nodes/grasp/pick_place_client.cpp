#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "moveit_planning/PickPlaceAction.h"
#include "moveit_planning/load_add_object.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_client");

    actionlib::SimpleActionClient<moveit_planning::PickPlaceAction> ac("pick_place", true);

    ROS_INFO("En attente du serveur d'action...");
    ac.waitForServer();
    std::string cube_ns = "cube1"; 
    std::string cube_id;

    //cube 1 saisir
    if (!ros::param::get("/" + cube_ns + "/id", cube_id)) {
        ROS_ERROR("Impossible de charger l'ID de l'objet : %s", cube_ns.c_str());
        return 1;
    }

    // Préparation du goal
    moveit_planning::PickPlaceGoal goal;
    goal.object_id = cube_id;  

    goal.goal_pose.header.frame_id = "world";
    goal.goal_pose.header.stamp = ros::Time::now();

    geometry_msgs::Pose cube_goal_pose = loadObjectPose("/goal"); 
    goal.goal_pose.pose = cube_goal_pose; 

    goal.solver_name = "OMPL"; 

    ROS_INFO("Envoi du goal Pick&Place pour l'objet: %s", goal.object_id.c_str());
    ac.sendGoal(goal);

    while (!ac.waitForResult(ros::Duration(1.0))) {
        auto feedback = ac.getResult();
        ROS_INFO("État actuel: %s", ac.getState().toString().c_str());
    }
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succès: %s", ac.getResult()->message.c_str());
    } else {
        ROS_ERROR("Échec: %s", ac.getResult()->message.c_str());
    }

    return 0;
}
