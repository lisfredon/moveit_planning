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
#include "moveit_planning/robot_utils.h"
#include "moveit_planning/moveTo_utils.h"
#include "moveit_planning/error_handling.h"
#include "moveit_planning/pick_place_manager.h"

#include <sstream>


int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PickPlaceManager manager("panda_manipulator", "panda_hand");

    // Ouvrir la pince au max dès le début
    manager.openGripper();

    // Charger la liste des cubes depuis YAML
    std::vector<std::string> cube_names;
    if (!nh.getParam("/cubes", cube_names)) {
        ROS_ERROR("Impossible de charger la liste des cubes (/cubes)");
        return 1;
    }
    // Ajouter tous les cubes dans la scène
    for (const auto& cube_ns : cube_names) {
        auto size = loadObjectSize("/" + cube_ns + "/size");
        auto pose = loadObjectPose("/" + cube_ns);
        auto id   = loadObjectID("/" + cube_ns + "/id");
        manager.addObject(id, pose, size);
    }

    // Cube cible à manipuler (param configurable)
    std::string object_id;
    nh.param<std::string>("/target_cube", object_id, "cube1");

    // Charger infos du cube cible
    auto objet_size = loadObjectSize("/" + object_id + "/size");
    auto objet_pose = loadObjectPose("/" + object_id);

    // Choisir la face pour le grasp
    GraspChoice grasp;
    if (!check(chooseGraspFace(nh, manager.getGripperGroup(), objet_size, grasp), "choix de la face de grasp")) return 1;

    //Visualisation des axes de l'objet pour debug 
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(object_id, 1, true);
    visualizeCubeFaces(marker_pub, objet_pose);

    // Phase d'approche
    if (!check(manager.approach(objet_pose, grasp.n_local, grasp.in_plane_axis), "phase d'approche")) return 1;
    
    //phase de grip
    if (!check(manager.grip(objet_pose, objet_size, grasp.n_local, grasp.in_plane_axis, grasp.face_index), "phase de grip")) return 1;
    
    geometry_msgs::PoseStamped eef_pose = manager.getArmGroup().getCurrentPose();

    tf2::Quaternion q_eef, q_obj;
    tf2::fromMsg(eef_pose.pose.orientation, q_eef);
    tf2::fromMsg(objet_pose.orientation, q_obj);

    // Orientation relative entre l’EEF et l’objet au moment du grasp
    tf2::Quaternion q_rel = q_eef.inverse() * q_obj;

    // Attacher l’objet
    manager.attachObject(object_id);

    // Déplacement vers le goal
    auto solver_goal = loadSolver("/goal_solver"); 
    auto goal_pose = loadObjectPose("/goal");

    // Normaliser orientation du YAML
    tf2::Quaternion q_obj_goal;
    tf2::fromMsg(goal_pose.orientation, q_obj_goal);
    q_obj_goal.normalize();

    // Recomposer orientation finale de l’EEF
    tf2::Quaternion q_eef_goal = q_obj_goal * q_rel.inverse();
    q_eef_goal.normalize();

    goal_pose.orientation = tf2::toMsg(q_eef_goal);

    if (!check(manager.moveToGoal(goal_pose, solver_goal), "phase déplacement vers l'objectif")) return 1;

    // Ouvrir la pince pour déposer l'objet
    manager.openGripper();

    // Détacher l’objet
    manager.detachObject(object_id);

    ros::shutdown();
}