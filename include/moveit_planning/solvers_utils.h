#ifndef SOLVERS_UTILS_H
#define SOLVERS_UTILS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <string>

enum class SolverType {
    OMPL,
    CARTESIAN,
    JOINT_INTERPOLATION
};

// Exécute un plan vers une pose avec le solveur choisi
bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group,
                    const geometry_msgs::Pose& target_pose,
                    SolverType solver);

#endif
