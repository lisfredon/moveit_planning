#include "moveit_planning/moveTo_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose_in,
    SolverType solver,
    const std::string& phase_name)
{

    tf2::Quaternion q;
    tf2::fromMsg(target_pose_in.orientation, q);
    q.normalize();
    geometry_msgs::Pose target_pose = target_pose_in;
    target_pose.orientation = tf2::toMsg(q);

    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }
    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}