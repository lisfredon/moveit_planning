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
#include <tf2/LinearMath/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>

#include "moveit_planning/moveTo_utils.h"

bool isFaceGraspable(const std::vector<double>& obj_size, int face_index, const std::string& side_face, double max_finger_opening);

//geometry_msgs::Pose generateAlignedGraspPose(const geometry_msgs::Pose& cube_pose);
geometry_msgs::Pose generateGraspPose(
    const geometry_msgs::Pose& cube_pose,
    const tf2::Vector3& n_local,
    const tf2::Vector3& grasp_in_plane_axis,
    double offset = 0.0);

double getFingerTarget(const std::vector<double>& cube_size, int face_index);

bool approch(moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::MoveGroupInterface& gripper_group,
        const geometry_msgs::Pose& obj_pose,
        const tf2::Vector3& n_local,
        const tf2::Vector3& in_plane_axis);

bool grip(moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::MoveGroupInterface& gripper_group,
        const geometry_msgs::Pose& obj_pose,
        const std::vector<double>& obj_size,
        const tf2::Vector3& n_local,
        const tf2::Vector3& in_plane_axis,
        int face_index);

struct GraspChoice {
    int face_index;
    std::string side_face;
    tf2::Vector3 n_local;
    tf2::Vector3 in_plane_axis;
};

bool chooseGraspFace(
    ros::NodeHandle& nh,
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    const std::vector<double>&  objet_size,
    GraspChoice& result);

#endif  // MOVEIT_PLANNING_GRASP_UTILS_H