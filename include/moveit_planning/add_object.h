#ifndef ADD_OBJECT_H
#define ADD_OBJECT_H

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <vector>
#include <string>
#include <moveit_msgs/CollisionObject.h> 
#include <string>

moveit_msgs::CollisionObject addObjectToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                              const std::string& object_id,
                                              const geometry_msgs::Pose& object_pose,
                                              const std::vector<double>& object_size,
                                              const std::string& frame_id = "panda_link");

#endif
