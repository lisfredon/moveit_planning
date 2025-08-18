#ifndef CLOSE_GRIPPER_UTILS_H
#define CLOSE_GRIPPER_UTILS_H

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <string>
#include <vector>


double getFingerTarget(const std::vector<double>& cube_size, int face_index);
void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double target);

#endif
