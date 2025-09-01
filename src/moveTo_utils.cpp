#include "moveit_planning/moveTo_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose_in,
    SolverType solver,
    const std::string& phase_name)
{
    // Normalisation de l’orientation demandée
    tf2::Quaternion q_goal;
    tf2::fromMsg(target_pose_in.orientation, q_goal);
    q_goal.normalize();

    // Orientation actuelle de l’EEF
    tf2::Quaternion q_current;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q_current);

    // Forcer la continuité : si produit scalaire < 0, inverser le quaternion
    if (q_goal.dot(q_current) < 0.0) {
        q_goal = tf2::Quaternion(-q_goal.x(), -q_goal.y(), -q_goal.z(), -q_goal.w());
    }

    // Construire la pose corrigée
    geometry_msgs::Pose target_pose = target_pose_in;
    target_pose.orientation = tf2::toMsg(q_goal);

    // Exécution
    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }
    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}

tf2::Quaternion computePlaceOrientation(
    int choice_face,                 // face du cube à mettre "en haut"
    const geometry_msgs::Pose& cube_pose_at_grasp // orientation initiale de l’objet
) {
    // normales locales
    std::vector<tf2::Vector3> normals = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };

    // normale de la face choisie (dans repère objet)
    tf2::Vector3 n_local = normals[choice_face];

    // quaternion du cube lors du grasp
    tf2::Quaternion q_obj;
    tf2::fromMsg(cube_pose_at_grasp.orientation, q_obj);

    // normale en repère monde (après grasp)
    tf2::Matrix3x3 R_obj(q_obj);
    tf2::Vector3 n_world = R_obj * n_local;

    // vecteur "vers le haut" désiré
    tf2::Vector3 desired_up(0, 0, 1);

    // rotation qui aligne n_world -> desired_up
    tf2::Quaternion q_align;
    q_align.setRotation(n_world.cross(desired_up), acos(n_world.dot(desired_up)));

    // nouvelle orientation finale de l’objet
    tf2::Quaternion q_final = q_align * q_obj;
    q_final.normalize();

    return q_final;
}
