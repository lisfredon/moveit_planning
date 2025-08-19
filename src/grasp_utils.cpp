#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>
#include "moveit_planning/grasp_utils.h"
#include <vector>
#include <cmath>

bool isFaceGraspable(const std::vector<double>& obj_size, int face_index, const std::string& side_face, double max_finger_opening) {
    double grasp_dim = 0.0;

    // Mapping des faces :
    // 0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z, 5:-Z
    switch(face_index) {
        case 0: // +X
        case 1: // -X
            // Plan YZ
            grasp_dim = (side_face == "width") ? obj_size[1] : obj_size[2];
            break;

        case 2: // +Y
        case 3: // -Y
            // Plan XZ
            grasp_dim = (side_face == "width") ? obj_size[0] : obj_size[2];
            break;

        case 4: // +Z
        case 5: // -Z
            // Plan XY
            grasp_dim = (side_face == "width") ? obj_size[0] : obj_size[1];
            break;

        default:
            ROS_ERROR("Face index invalide !");
            return false;
    }

    ROS_INFO_STREAM("Face " << face_index 
                    << " avec side_face=" << side_face 
                    << " → dimension testée=" << grasp_dim 
                    << ", ouverture max pince=" << max_finger_opening);

    return grasp_dim <= max_finger_opening;
}



double getMaxFingerOpening(moveit::planning_interface::MoveGroupInterface& gripper_group) {
    auto joint_model_group = gripper_group.getRobotModel()->getJointModelGroup(gripper_group.getName());
    const std::vector<const moveit::core::JointModel*>& joints = joint_model_group->getActiveJointModels();

    double max_opening = 0.0;
    for (const auto& joint : joints) {
        const auto& bounds = joint->getVariableBounds(joint->getName());
        if (bounds.position_bounded_) {
            double joint_max = bounds.max_position_;
            if (joint_max > max_opening) max_opening = joint_max;
        }
    }
    return max_opening*2;
}


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


double getFingerTarget(const std::vector<double>& cube_size, int face_index) {
    switch(face_index) {
        case 0: case 1: return cube_size[0] / 2.0;
        case 2: case 3: return cube_size[1] / 2.0;
        case 4: case 5: return cube_size[2] / 2.0;
    }
    return 0.0;
}

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double target) {
    std::vector<double> gripper_targets = {target, target};
    gripper_group.setJointValueTarget(gripper_targets);
    gripper_group.move();
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

bool grip(moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::MoveGroupInterface& gripper_group,
        const geometry_msgs::Pose& obj_pose,
        const std::vector<double>& obj_size,
        const tf2::Vector3& n_local,
        const tf2::Vector3& in_plane_axis,
        int face_index)
{
    if (!moveToGraspPhase(move_group, obj_pose, n_local, in_plane_axis, 0.0, SolverType::OMPL, "grip")) return false;
    
    //Fermer la pince
    double finger_target = getFingerTarget(obj_size, face_index);
    closeGripper(gripper_group, finger_target);
    ROS_INFO_STREAM("Fermeture du grip reussie");
    return true;
}