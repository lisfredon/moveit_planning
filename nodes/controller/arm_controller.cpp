#include "moveit_planning/arm_controller.h"
#include "moveit_planning/moveTo_utils.h"
#include "moveit_planning/grasp_utils.h"
#include <ros/ros.h>

ArmController::ArmController(const std::string& arm_group_name)
    : arm_group_(arm_group_name) {}

bool ArmController::moveToPose(const geometry_msgs::Pose& pose,
                                SolverType solver_name)
{
    return ::moveTo(arm_group_, pose, solver_name, "moveToPose");
}

bool ArmController::approachObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                                   const geometry_msgs::Pose& obj_pose,
                                   const tf2::Vector3& n_local,
                                   const tf2::Vector3& in_plane_axis)
{
    return ::approch(arm_group_, gripper_group, obj_pose, n_local, in_plane_axis);
}

bool ArmController::gripObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                               const geometry_msgs::Pose& obj_pose,
                               const std::vector<double>& obj_size,
                               const tf2::Vector3& n_local,
                               const tf2::Vector3& in_plane_axis,
                               int face_index)
{
    return ::grip(arm_group_, gripper_group, obj_pose, obj_size, n_local, in_plane_axis, face_index);
}

moveit::planning_interface::MoveGroupInterface& ArmController::getMoveGroup()
{
    return arm_group_;
}
