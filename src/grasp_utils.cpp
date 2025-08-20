#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>

#include <vector>
#include <cmath>
#include <random>

#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/robot_utils.h"

bool isFaceGraspable(const std::vector<double>& obj_size, int face_index, const std::string& side_face, double max_finger_opening) {
    double grasp_dim = 0.0;

    // Mapping des faces :
    // 0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z, 5:-Z
    switch(face_index) {
        case 0: // +X
        case 1: // -X
            // Plan YZ
            grasp_dim = (side_face == "width_high" || side_face == "width_down") ? obj_size[1] : obj_size[2];
            break;

        case 2: // +Y
        case 3: // -Y
            // Plan XZ
            grasp_dim = (side_face == "width_high" || side_face == "width_down") ? obj_size[0] : obj_size[2];
            break;

        case 4: // +Z
        case 5: // -Z
            // Plan XY
            grasp_dim = (side_face == "width_high" || side_face == "width_down") ? obj_size[0] : obj_size[1];
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

bool approch(moveit::planning_interface::MoveGroupInterface& move_group,
        moveit::planning_interface::MoveGroupInterface& gripper_group,
        const geometry_msgs::Pose& obj_pose,
        const tf2::Vector3& n_local,
        const tf2::Vector3& in_plane_axis)
{
    geometry_msgs::Pose target_pose = generateGraspPose(obj_pose, n_local, in_plane_axis, 0.05);
    if (!moveTo(move_group, target_pose, SolverType::OMPL, "approche")) return 1;
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
    geometry_msgs::Pose target_pose = generateGraspPose(obj_pose, n_local, in_plane_axis, 0.0);
    if (!moveTo(move_group, target_pose, SolverType::OMPL, "préhension")) return false;
    
    //Fermer la pince
    double finger_target = getFingerTarget(obj_size, face_index);
    closeGripper(gripper_group, finger_target);
    ROS_INFO_STREAM("Fermeture du grip reussie");
    return true;
}

bool chooseGraspFace(
    ros::NodeHandle& nh,
    moveit::planning_interface::MoveGroupInterface& gripper_group,
    const std::vector<double>& objet_size,
    GraspChoice& result)
{
    // Lire paramètres YAML
    int face_index_param;
    std::string side_face_param;
    bool has_face = nh.getParam("/how_take_cube/choice_face", face_index_param);
    bool has_side = nh.getParam("/how_take_cube/side_face", side_face_param);

    std::vector<int> faces;
    std::vector<std::string> all_sides = {"width_high", "width_down", "length_high", "length_down"}
;

    if (has_face) {
        faces = {face_index_param};
    } else {
        faces = {0,1,2,3,4,5};
        std::shuffle(faces.begin(), faces.end(), std::mt19937(std::random_device{}()));
        ROS_WARN("Aucune face_index fournie → test aléatoire des faces.");
    }

    std::vector<std::string> sides;
    if (has_side) {
        sides = {side_face_param};
    } else {
        sides = all_sides;
        std::shuffle(sides.begin(), sides.end(), std::mt19937(std::random_device{}()));
        ROS_WARN("Aucun side_face fourni → test aléatoire entre width_high', 'width_down', 'length_high' et'length_down'.");
    }

    double max_opening = getMaxFingerOpening(gripper_group);
    // On met toutes les combinaisons dans une file
    std::vector<std::pair<int,std::string>> candidates;
    for (int f : faces)
        for (auto& s : sides)
            candidates.emplace_back(f,s);

    // Tant qu’il reste des candidats
    while (!candidates.empty()) {
        auto [face_index, side_face] = candidates.back();
        candidates.pop_back();

        ROS_INFO_STREAM(">>> Test face " << face_index 
                        << " avec side=" << side_face);

        // Vérification géométrique
        if (!isFaceGraspable(objet_size, face_index, side_face, 0.07)) {
            ROS_WARN("Face non saisissable, on passe à la suivante.");
            continue;
        }

        // Succès total → on retourne
        result.face_index = face_index;
        result.side_face = side_face;
        result.n_local = getNormalObject(face_index);
        result.in_plane_axis = getObjectAxis(face_index, side_face);

        ROS_INFO_STREAM(">>> Face choisie = " << face_index 
                        << " avec side=" << side_face);
        return true;
    }

    ROS_ERROR("Aucune combinaison faisable !");
    return false;
}