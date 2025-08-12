/*#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H

#include <geometry_msgs/Pose.h>
#include <string>

geometry_msgs::Pose createGraspPose(const geometry_msgs::Pose& object_pose,
                                    const std::string& strategy);

#endif

#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

std::vector<geometry_msgs::Pose> generateGraspPoses(
    const geometry_msgs::Pose& base_pose,
    const Eigen::Vector3d& axis,
    double angle_step,
    double max_angle,
    const tf2::Quaternion& orientation);

#endif // GRASP_UTILS_H
*/


#ifndef MOVEIT_PLANNING_GRASP_UTILS_H
#define MOVEIT_PLANNING_GRASP_UTILS_H

#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::Pose generateAlignedGraspPose(const geometry_msgs::Pose& cube_pose);
#endif  // MOVEIT_PLANNING_GRASP_UTILS_H