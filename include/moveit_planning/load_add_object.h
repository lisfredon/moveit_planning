#ifndef LOAD_ADD_OBJECT_H
#define LOAD_ADD_OBJECT_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <vector>
#include <string>
#include <moveit_msgs/CollisionObject.h> 
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

geometry_msgs::Pose loadObjectPose(const std::string& param_namespace);

std::vector<double> loadObjectSize(const std::string& param_name);

std::string loadObjectID(const std::string& param_name);
std::string loadSideFace(const std::string& param_name);

int loadGoalFace(const std::string& param_name);

moveit_msgs::CollisionObject addObjectToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                              const std::string& object_id,
                                              const geometry_msgs::Pose& object_pose,
                                              const std::vector<double>& object_size,
                                              const std::string& frame_id = "panda_link");


tf2::Vector3 getNormalObject(int face_index);

tf2::Vector3 getObjectAxis(int face_index, const std::string& side_face);

#endif
