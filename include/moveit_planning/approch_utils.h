#ifndef APPROACH_UTILS_H
#define APPROACH_UTILS_H

#include <geometry_msgs/Pose.h>
#include <string>

enum class ApproachType {
    VERTICALE,
    HORIZONTALE,
    DIAGONALE
};

ApproachType approachTypeFromString(const std::string& type_str);

geometry_msgs::Pose createApproachPose(const geometry_msgs::Pose& object_pose,
                                       ApproachType type,
                                       double offset);

#endif
