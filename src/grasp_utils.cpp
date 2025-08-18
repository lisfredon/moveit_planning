#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>
#include "moveit_planning/grasp_utils.h"
#include <vector>
#include <cmath>

geometry_msgs::Pose generateGraspPose(
    const geometry_msgs::Pose& cube_pose,
    const tf2::Vector3& n_local,
    const tf2::Vector3& in_plane_axis_local,
    double offset)
{
    // Rotation réelle du cube
    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose.orientation, q_cube);
    tf2::Matrix3x3 R_cube(q_cube);

    // Normale et axe dans le plan en coordonnées globales
    tf2::Vector3 z_axis = (R_cube * n_local).normalized();
    tf2::Vector3 y_axis = (R_cube * in_plane_axis_local).normalized();

    // Si y_axis est trop colinéaire à z_axis → on choisit un vecteur fixe
    if (fabs(y_axis.dot(z_axis)) > 0.999) {
        // Vecteur global fixe pour stabiliser ±Z
        tf2::Vector3 world_ref(0, 1, 0);
        if (fabs(world_ref.dot(z_axis)) > 0.999) {
            world_ref = tf2::Vector3(1, 0, 0);
        }
        y_axis = (world_ref - world_ref.dot(z_axis) * z_axis).normalized();
    }

    // Orthogonaliser
    tf2::Vector3 x_axis = y_axis.cross(z_axis).normalized();
    y_axis = z_axis.cross(x_axis).normalized();

    // Position de préhension
    tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
    tf2::Vector3 p_gripper = p_cube - offset * z_axis;

    // Rotation finale
    tf2::Matrix3x3 R_gripper(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());

    tf2::Quaternion q_gripper;
    R_gripper.getRotation(q_gripper);

    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = p_gripper.x();
    grasp_pose.position.y = p_gripper.y();
    grasp_pose.position.z = p_gripper.z();
    grasp_pose.orientation = tf2::toMsg(q_gripper);

    return grasp_pose;
}

bool moveToGraspPhase(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& obj_pose,
    const tf2::Vector3& normal,
    const tf2::Vector3& in_plane,
    double offset,
    SolverType solver,
    const std::string& phase_name)
{
    geometry_msgs::Pose target_pose = generateGraspPose(obj_pose, normal, in_plane, offset);
    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }
    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}