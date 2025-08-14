#include "moveit_planning/load_utils.h"

// Charger une pose depuis le paramètre ROS
geometry_msgs::Pose loadParam(const std::string& param_namespace) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;

    bool pos_ok = ros::param::get(param_namespace + "/position", pos);
    bool ori_ok = ros::param::get(param_namespace + "/orientation", ori);

    if (!pos_ok) {
        ROS_ERROR_STREAM("Position manquante pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }
    if (!ori_ok) {
        ROS_ERROR_STREAM("Orientation manquante pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }

    // Compléter la position à 3 valeurs avec des 0.0
    if (pos.size() < 3) {
        pos.resize(3, 0.0);
        ROS_WARN_STREAM("Position incomplète pour " << param_namespace
                        << " -> complétée à : [" 
                        << pos[0] << ", " << pos[1] << ", " << pos[2] << "]");
    }

    // Compléter l'orientation à 4 valeurs avec des 0.0 et un w = 1.0 par défaut
    if (ori.size() < 4) {
        while (ori.size() < 3) ori.push_back(0.0);
        if (ori.size() == 3) ori.push_back(1.0); // w par défaut
        ROS_WARN_STREAM("Orientation incomplète pour " << param_namespace
                        << " -> complétée à : [" 
                        << ori[0] << ", " << ori[1] << ", " << ori[2] << ", " << ori[3] << "]");
    }

    // Affecter à la pose
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];

    return pose;
}


std::vector<double> loadObjectSizeParam(const std::string& param_name) {
    std::vector<double> size;
    if (!ros::param::get(param_name, size) || size.size() != 3) {
        ROS_ERROR_STREAM("Impossible de charger la taille du cube : " << param_name);
        return {};
    }
    return size;
}