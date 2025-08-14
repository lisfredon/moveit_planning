#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <string>

void publishFaceNormalsWithText(const geometry_msgs::Pose& cube_pose,
                                const std::vector<tf2::Vector3>& face_normals,
                                const std::vector<std::string>& face_names,
                                ros::Publisher& marker_pub);
