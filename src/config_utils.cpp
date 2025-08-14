#include "moveit_planning/config_utils.h"
#include <ros/ros.h>

geometry_msgs::Pose loadPoseParam(const std::string& ns) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;
    if (!ros::param::get(ns + "/position", pos) ||
        !ros::param::get(ns + "/orientation", ori)) {
        ROS_ERROR_STREAM("Impossible de charger les paramètres pour " << ns);
        throw std::runtime_error("Paramètres manquants");
    }
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];
    return pose;
}

GraspConfig loadConfigFromParams() {
    GraspConfig cfg;
    cfg.face_normals = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };
    cfg.face_names = {"+X","-X","+Y","+Y","+Z","-Z"};
    ros::param::get("/cube/size", cfg.cube_size);
    cfg.cube_pose = loadPoseParam("/cube");
    return cfg;
}
