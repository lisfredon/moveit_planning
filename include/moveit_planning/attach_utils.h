#ifndef ATTACH_UTILS_H
#define ATTACH_UTILS_H

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h> 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <string>
#include <vector>

void attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  const moveit_msgs::CollisionObject& object);
#endif
