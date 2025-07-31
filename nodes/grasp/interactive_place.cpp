#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "interactive_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    move_group.setPlanningTime(20.0);
    move_group.setPoseReferenceFrame("panda_link0");

    // Récupérer la pose de saisie depuis le paramètre ROS
    XmlRpc::XmlRpcValue pose_param;
    if (!nh.getParam("/grasp/last_grasp_pose", pose_param) || pose_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Paramètre /grasp/last_grasp_pose invalide ou introuvable.");
        return 1;
    }

    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = static_cast<double>(pose_param["position"]["x"]);
    grasp_pose.position.y = static_cast<double>(pose_param["position"]["y"]);
    grasp_pose.position.z = static_cast<double>(pose_param["position"]["z"]);
    grasp_pose.orientation.x = static_cast<double>(pose_param["orientation"]["x"]);
    grasp_pose.orientation.y = static_cast<double>(pose_param["orientation"]["y"]);
    grasp_pose.orientation.z = static_cast<double>(pose_param["orientation"]["z"]);
    grasp_pose.orientation.w = static_cast<double>(pose_param["orientation"]["w"]);

    // Lire la position de dépôt
    double px, py, pz;
    std::cout << "Position de pose (x y z) : ";
    std::cin >> px >> py >> pz;

    // Choisir l'orientation à la pose
    std::string orientation_choice;
    std::cout << "Orientation à la pose ('face_robot' ou 'face_haut') : ";
    std::cin >> orientation_choice;

    tf2::Quaternion original_q, rotation_offset_q;
    tf2::fromMsg(grasp_pose.orientation, original_q);

    if (orientation_choice == "face_haut") {
        rotation_offset_q.setRPY(0, M_PI/2, 0); // ajustable selon ton repère
    } else {
        rotation_offset_q.setRPY(0, 0, 0); // pas de rotation
    }

    tf2::Quaternion final_q = rotation_offset_q * original_q;
    final_q.normalize();

    geometry_msgs::Pose place_pose;
    place_pose.position.x = px;
    place_pose.position.y = py;
    place_pose.position.z = pz;
    place_pose.orientation = tf2::toMsg(final_q);

    // Définir la cible et exécuter
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(place_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        ROS_INFO("Objet posé.");
    } else {
        ROS_ERROR("Échec du plan de pose.");
        return 1;
    }

    return 0;
}
