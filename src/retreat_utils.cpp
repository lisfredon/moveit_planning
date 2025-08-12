#include "moveit_planning/retreat_utils.h"

geometry_msgs::Pose createRetreatPose(const geometry_msgs::Pose& grasp_pose,
                                      double retreat_distance) {
    geometry_msgs::Pose retreat_pose = grasp_pose;
    retreat_pose.position.z += retreat_distance;
    return retreat_pose;
}
