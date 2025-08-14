#include "moveit_planning/load_utils.h"

// Charger une pose depuis le paramètre ROS
geometry_msgs::Pose loadParam(const std::string& param_namespace) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;
    if (!ros::param::get(param_namespace + "/position", pos) ||
        !ros::param::get(param_namespace + "/orientation", ori)) {
        ROS_ERROR_STREAM("Impossible de charger les paramètres pour " << param_namespace);
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

std::vector<double> loadCubeSizeParam(const std::string& param_name) {
    std::vector<double> size;
    if (!ros::param::get(param_name, size) || size.size() != 3) {
        ROS_ERROR_STREAM("Impossible de charger la taille du cube : " << param_name);
        return {};
    }
    return size;
}