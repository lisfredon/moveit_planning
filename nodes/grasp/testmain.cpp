#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>
#include <vector>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include "moveit_planning/solvers_utils.h"
#include "moveit_planning/visualization_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/attach_utils.h"
#include "moveit_planning/close_gripper_utils.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Charger cube
    auto cube_size = loadObjectSize("/cube/size");
    auto cube_pose = loadObjectPose("/cube");

    // Ajouter cube à MoveIt
    auto cube = addObjectToScene(planning_scene_interface, "cube", cube_pose, cube_size, move_group.getPlanningFrame());

    // Choix de la face et de l'orientation dans le plan
    int face_index = 5; // 0:+X, 1:-X, 2:+Y, ...
    tf2::Vector3 n_local = getNormalObject(face_index);
    tf2::Vector3 in_plane_axis = getObjectAxis(face_index, false);// true = "largeur", false = "longueur"
    
    //Visualisation des axes du cube pour debug 
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cube", 1, true);
    visualizeCubeFaces(marker_pub, cube_pose);

    // Phase d'approche
    if (!moveToGraspPhase(move_group, cube_pose, n_local, in_plane_axis, 0.05, SolverType::OMPL, "approche")) return 1;

    //phase de grip
    if (!moveToGraspPhase(move_group, cube_pose, n_local, in_plane_axis, 0.0, SolverType::OMPL, "grip")) return 1;


    double finger_target = getFingerTarget(cube_size, face_index);
    // Fermer la pince
    closeGripper(gripper_group, finger_target);

    // Attacher l’objet
    attachObject(planning_scene_interface, move_group, cube);

    ros::shutdown();
}