#ifndef MOVE_TO_GOAL_H
#define MOVE_TO_GOAL_H

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

bool moveToGoal(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& goal_pose);

#endif
