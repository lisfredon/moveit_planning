#ifndef MOVEIT_PLANNING_LOAD_UTILS_H
#define MOVEIT_PLANNING_LOAD_UTILS_H

#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <ros/ros.h>

geometry_msgs::Pose loadParam(const std::string& param_namespace);

std::vector<double> loadObjectSizeParam(const std::string& param_name);

#endif