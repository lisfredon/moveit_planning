#ifndef MOVE_TO_GOAL_H
#define MOVE_TO_GOAL_H

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "moveit_planning/solvers_utils.h"

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose,
    SolverType solver,
    const std::string& phase_name);

#endif
