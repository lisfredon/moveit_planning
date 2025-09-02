#include "moveit_planning/moveTo_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose_in,
    SolverType solver,
    const std::string& phase_name)
{
    // Normalisation de l’orientation demandée
    tf2::Quaternion q_goal;
    tf2::fromMsg(target_pose_in.orientation, q_goal);
    q_goal.normalize();

    // Orientation actuelle de l’EEF
    tf2::Quaternion q_current;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q_current);

    // Forcer la continuité : si produit scalaire < 0, inverser le quaternion
    if (q_goal.dot(q_current) < 0.0) {
        q_goal = tf2::Quaternion(-q_goal.x(), -q_goal.y(), -q_goal.z(), -q_goal.w());
    }

    // Construire la pose corrigée
    geometry_msgs::Pose target_pose = target_pose_in;
    target_pose.orientation = tf2::toMsg(q_goal);

    // Exécution
    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }
    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}