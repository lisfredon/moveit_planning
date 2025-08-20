#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h> 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <string>
#include <vector>

bool attachObject(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& object_id,
    const std::string& hand_link,
    const std::vector<std::string>& touch_links);

void openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group);

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double target);

double getMaxFingerOpening(moveit::planning_interface::MoveGroupInterface& gripper_group);

void detachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  const std::string& object_id);

#endif
