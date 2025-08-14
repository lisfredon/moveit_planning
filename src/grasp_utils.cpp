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
    const tf2::Vector3& grasp_in_plane_axis,
    double offset){
        tf2::Quaternion q_cube;
        tf2::fromMsg(cube_pose.orientation, q_cube);
        tf2::Matrix3x3 R_cube(q_cube);

        tf2::Vector3 n_global = R_cube * n_local;
        tf2::Vector3 grasp_axis_global = R_cube * grasp_in_plane_axis;

        tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
        tf2::Vector3 p_gripper = p_cube - offset * n_global;

        // Construire la base du gripper
        tf2::Vector3 z_axis = n_global.normalized();
        tf2::Vector3 y_axis = (grasp_axis_global - (grasp_axis_global.dot(z_axis)) * z_axis).normalized();
        tf2::Vector3 x_axis = y_axis.cross(z_axis).normalized();

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