#ifndef RETREAT_UTILS_H
#define RETREAT_UTILS_H

#include <geometry_msgs/Pose.h>

geometry_msgs::Pose createRetreatPose(const geometry_msgs::Pose& grasp_pose,
                                      double retreat_distance);

#endif
