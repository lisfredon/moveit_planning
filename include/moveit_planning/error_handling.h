#pragma once
#include <ros/ros.h>
#include <string>

inline bool check(bool success, const std::string& step) {
    if (!success) {
        ROS_ERROR_STREAM("Échec à l'étape : " << step);
        ros::shutdown();
        return false;
    }
    return true;
}
