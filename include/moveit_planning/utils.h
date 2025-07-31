#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H

#include <string>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Pose createGraspPose(const std::string& direction, const geometry_msgs::Pose& object_pose);

#endif  // GRASP_UTILS_H
