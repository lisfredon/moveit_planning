#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pr2_right_arm_move_cpp");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Interface pour le bras droit
    moveit::planning_interface::MoveGroupInterface right_arm("right_arm");

    // Affiche les joints et l'effecteur
    std::vector<std::string> joints = right_arm.getJoints();
    ROS_INFO("Joints du bras droit:");
    for (const auto& joint : joints)
        ROS_INFO("%s", joint.c_str());

    ROS_INFO("Effecteur: %s", right_arm.getEndEffectorLink().c_str());

    // Vitesse et acceleration
    right_arm.setMaxVelocityScalingFactor(0.5);
    right_arm.setMaxAccelerationScalingFactor(0.5);

    // Déplacement cartésien
    geometry_msgs::Pose pose_target;
    pose_target.position.x = 0.5;
    pose_target.position.y = -0.2;
    pose_target.position.z = 1.0;
    pose_target.orientation.w = 1.0;

    right_arm.setPoseTarget(pose_target);

    // Planification et exécution
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (right_arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Planification réussie. Exécution...");
        right_arm.move();
        ROS_INFO("Déplacement terminé");
    }
    else
    {
        ROS_WARN("Impossible de trouver un plan valide pour le bras droit.");
    }

    ros::shutdown();
    return 0;
}
