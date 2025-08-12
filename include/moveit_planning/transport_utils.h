#ifndef TRANSPORT_UTILS_H
#define TRANSPORT_UTILS_H

#include <geometry_msgs/Pose.h>

geometry_msgs::Pose createTransportPose(const geometry_msgs::Pose& grasp_pose,
                                        const geometry_msgs::Pose& goal_pose);

#endif
