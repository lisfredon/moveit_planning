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

/// Crée une pose de préhension à partir d'une stratégie
geometry_msgs::Pose createGraspPose(const geometry_msgs::Pose& object_pose,
                                    const std::string& strategy);

/// Génère des poses en tournant autour d’un axe
std::vector<geometry_msgs::Pose> generateGraspPoses(
    const geometry_msgs::Pose& object_pose,
    const Eigen::Vector3d& rotation_axis = Eigen::Vector3d::UnitZ(),
    double angle_delta = M_PI / 8.0,
    double max_angle = 2 * M_PI,
    const tf2::Quaternion& orientation_offset = tf2::Quaternion(0, 0, 0, 1));

/// Génère une pose pour chaque face du cube (en orientant la pince face normale)
std::vector<geometry_msgs::Pose> generateFaceGraspPoses(const geometry_msgs::Pose& cube_pose_msg);

#endif  // MOVEIT_PLANNING_GRASP_UTILS_H