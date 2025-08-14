#pragma once
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>

struct GraspConfig {
    double approach_offset = 0.05;
    double grip_offset = 0.0;
    std::string arm_group = "panda_manipulator";
    std::string gripper_group = "panda_hand";
    std::vector<tf2::Vector3> face_normals;
    std::vector<std::string> face_names;
    geometry_msgs::Pose cube_pose;
    std::vector<double> cube_size;
};

GraspConfig loadConfigFromParams();
