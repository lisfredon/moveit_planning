#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <iostream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "interactive_spawn_cube");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string answer;
    std::cout << "Veux-tu ajouter un cube ? (oui/non) : ";
    std::getline(std::cin, answer);

    if (answer != "oui" && answer != "Oui") {
        ROS_INFO("Pas d'ajout de cube, fin du programme.");
        return 0;
    }

    double px, py, pz;
    std::cout << "Entre la position du cube (x y z) séparés par espace : ";
    std::cin >> px >> py >> pz;

    double sx = 0.05, sy = 0.05, sz = 0.05;  // taille fixe de la cube

    // Création de l'objet collision cube
    moveit_msgs::CollisionObject cube;
    cube.id = "cube";
    cube.header.frame_id = "panda_link0";  // Cadre de référence robot

    // Définir la forme primitive (box)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = sx;
    primitive.dimensions[1] = sy;
    primitive.dimensions[2] = sz;

    cube.primitives.push_back(primitive);

    // Pose de la cube
    geometry_msgs::Pose cube_pose;
    cube_pose.position.x = px;
    cube_pose.position.y = py;
    cube_pose.position.z = pz;
    //cube_pose.orientation.w = 1.0; // Pas de rotation
    cube_pose.orientation.x = 0.7;
    cube_pose.orientation.y = 0.0;
    cube_pose.orientation.z = 0.5;
    cube_pose.orientation.w = 1.0;

    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cube);

    // Ajouter la cube à la scène
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("cube ajoutée à la scène avec position (%.2f, %.2f, %.2f) et taille (%.2f, %.2f, %.2f).",
             px, py, pz, sx, sy, sz);

    // Enregistrer la position de l'objet pour interactive_grasp
    nh.setParam("/grasp/obj_pos_x", px);
    nh.setParam("/grasp/obj_pos_y", py);
    nh.setParam("/grasp/obj_pos_z", pz); 
    ROS_INFO("Position de l'objet enregistrée dans les paramètres ROS.");

    nh.setParam("/grasp/obj_ori_x", 0.0);
    nh.setParam("/grasp/obj_ori_y", 0.0);
    nh.setParam("/grasp/obj_ori_z", 0.5);
    nh.setParam("/grasp/obj_ori_w", 1.0);
    ROS_INFO("Orientation de l'objet enregistrée dans les paramètres ROS.");

    
    ros::waitForShutdown();
    return 0;
}
