#include "moveit_planning/approch_utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ApproachType approachTypeFromString(const std::string& type_str) {
    if (type_str == "verticale") return ApproachType::VERTICALE;
    if (type_str == "horizontale") return ApproachType::HORIZONTALE;
    return ApproachType::DIAGONALE;
}

geometry_msgs::Pose createApproachPose(const geometry_msgs::Pose& object_pose,
                                       ApproachType type,
                                       double offset) {
    geometry_msgs::Pose approach_pose = object_pose;
    tf2::Quaternion q;

    switch (type) {
        case ApproachType::VERTICALE:
            approach_pose.position.z += offset;
            q.setRPY(M_PI, 0, 0);
            break;
        case ApproachType::HORIZONTALE:
            approach_pose.position.x -= offset;
            q.setRPY(0, M_PI/2, 0);
            break;
        case ApproachType::DIAGONALE:
            approach_pose.position.x -= offset / 1.4;
            approach_pose.position.z += offset / 1.4;
            q.setRPY(M_PI/4, 0, 0);
            break;
    }

    approach_pose.orientation = tf2::toMsg(q);
    return approach_pose;
}
