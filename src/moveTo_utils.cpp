#include "moveit_planning/moveTo_utils.h"

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose,
    SolverType solver,
    const std::string& phase_name)
{
    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }
    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}