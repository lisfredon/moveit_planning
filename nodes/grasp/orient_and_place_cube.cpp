#include <ros/ros.h>

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "moveit_planning/utils.h"

// Fonction pour comparer deux couleurs (avec marge d'erreur)
bool colorsAreEqual(const std::vector<double>& c1, const std::vector<double>& c2, double epsilon = 0.05) {
    return std::fabs(c1[0] - c2[0]) < epsilon &&
           std::fabs(c1[1] - c2[1]) < epsilon &&
           std::fabs(c1[2] - c2[2]) < epsilon;
}

geometry_msgs::Quaternion rotateCubeToShowFace(const std::string& face) {
    tf2::Quaternion q;

    if (face == "front") {
        q.setRPY(0, 0, 0);  // Pas besoin de rotation
    } else if (face == "back") {
        q.setRPY(0, M_PI, 0);  // Rotation de 180° autour de Y
    } else if (face == "left") {
        q.setRPY(0, -M_PI / 2, 0);  // Rotation de -90° autour de Y
    } else if (face == "right") {
        q.setRPY(0, M_PI / 2, 0);  // Rotation de +90° autour de Y
    } else if (face == "top") {
        q.setRPY(-M_PI / 2, 0, 0);  // Faire passer le top en face
    } else if (face == "bottom") {
        q.setRPY(M_PI / 2, 0, 0);   // Faire passer le bottom en face
    } else {
        ROS_WARN("Face inconnue pour réorientation : %s", face.c_str());
        q.setRPY(0, 0, 0);  // par défaut
    }

    q.normalize();
    geometry_msgs::Quaternion orientation;
    tf2::convert(q, orientation);
    return orientation;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orient_and_place_cube");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Lire les couleurs depuis le paramètre serveur
    std::vector<std::string> faces = {"front", "back", "left", "right", "top", "bottom"};
    std::map<std::string, std::vector<double>> face_colors;

    for (const auto& face : faces) {
        std::vector<double> color;
        if (nh.getParam("/grasp/cube/face_colors/" + face, color) && color.size() == 3) {
            face_colors[face] = color;
        } else {
            ROS_WARN("Couleur non trouvée pour la face '%s'", face.c_str());
        }
    }

    // Correspondance entre noms de couleur (texte) et valeurs RGB
    std::map<std::string, std::vector<double>> color_name_map = {
        {"rouge",   {1.0, 0.0, 0.0}},
        {"vert",    {0.0, 1.0, 0.0}},
        {"bleu",    {0.0, 0.0, 1.0}},
        {"jaune",   {1.0, 1.0, 0.0}},
        {"cyan",    {0.0, 1.0, 1.0}},
        {"magenta", {1.0, 0.0, 1.0}}
    };

    std::string saisie_couleur;
    std::cout << "Quelle couleur veux-tu saisir ? (rouge, vert, bleu, jaune, cyan, magenta) : ";
    std::cin >> saisie_couleur;

    if (color_name_map.find(saisie_couleur) == color_name_map.end()) {
        ROS_ERROR("Couleur inconnue : %s", saisie_couleur.c_str());
        return 1;
    }

    auto couleur_recherchee = color_name_map[saisie_couleur];

    // Rechercher à quelle face cette couleur correspond
    std::string face_correspondante;
    bool trouve = false;
    for (const auto& pair : face_colors) {
        const std::string& face = pair.first;
        const std::vector<double>& couleur = pair.second;

        if (colorsAreEqual(couleur, couleur_recherchee)) {
            face_correspondante = face;
            ROS_INFO("La couleur %s correspond à la face : %s", 
                     saisie_couleur.c_str(), face.c_str());
            trouve = true;
            break;
        }
    }

    if (!trouve) {
        ROS_WARN("Aucune face ne correspond exactement à la couleur %s.", saisie_couleur.c_str());
        return 1;
    }
    
    // Choix de la stratégie de saisie
    std::vector<std::string> faces_paralleles_X = {"front", "back", "top", "bottom"};
    std::string direction = "H_parallele";

    // Lire la position de l'objet depuis les paramètres ROS
    double ox, oy, oz;
    if (!nh.getParam("/grasp/cube/position/x", ox) ||
        !nh.getParam("/grasp/cube/position/y", oy) ||
        !nh.getParam("/grasp/cube/position/z", oz)) {
        ROS_ERROR("Impossible de récupérer la position de l'objet depuis les paramètres ROS.");
        return 1;
    }

    geometry_msgs::Pose object_pose;
    object_pose.position.x = ox;
    object_pose.position.y = oy;
    object_pose.position.z = oz;
    object_pose.orientation.w = 1.0;

    geometry_msgs::Pose grasp_pose = createGraspPose(direction, object_pose);
    grasp_pose.position.z += 0.1;

    // Planification avec MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    move_group.setPoseTarget(grasp_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
    } else {
        ROS_WARN("Planification échouée.");
    }

    object_pose.orientation = rotateCubeToShowFace(face_correspondante);


    return 0;
}
