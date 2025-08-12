#include "moveit_planning/transport_utils.h"
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose createTransportPose(const geometry_msgs::Pose& grasp_pose,
                                        const geometry_msgs::Pose& goal_pose) {
    return goal_pose; // Peut être enrichi (ex : interpolations)
}
