#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>

#include "moveit_planning/solvers_utils.h"

class ArmController {
public:
    ArmController(const std::string& arm_group_name);

    bool moveToPose(const geometry_msgs::Pose& pose, SolverType solver_name);
    bool approachObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                        const geometry_msgs::Pose& obj_pose,
                        const tf2::Vector3& n_local,
                        const tf2::Vector3& in_plane_axis);

    bool gripObject(moveit::planning_interface::MoveGroupInterface& gripper_group,
                    const geometry_msgs::Pose& obj_pose,
                    const std::vector<double>& obj_size,
                    const tf2::Vector3& n_local,
                    const tf2::Vector3& in_plane_axis,
                    int face_index);


    moveit::planning_interface::MoveGroupInterface& getMoveGroup();

private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
};
