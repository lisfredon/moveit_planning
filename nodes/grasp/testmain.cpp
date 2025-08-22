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

    // Charger objet
    auto objet_size = loadObjectSize("/cube/size");
    auto objet_pose = loadObjectPose("/cube");
    auto object_id = loadObjectID("/cube/id");

    // Ajouter objet à MoveIt
    auto object = manager.addObject(object_id, objet_pose, objet_size);

    GraspChoice grasp;
    if (!check(chooseGraspFace(nh, manager.getGripperGroup(), objet_size, grasp), "choix de la face de grasp")) return 1;

    //Visualisation des axes de l'objet pour debug 
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(object_id, 1, true);
    visualizeCubeFaces(marker_pub, objet_pose);

    // Phase d'approche
    if (!check(manager.approach(objet_pose, grasp.n_local, grasp.in_plane_axis), "phase d'approche")) return 1;
    
    //phase de grip
    if (!check(manager.grip(objet_pose, objet_size, grasp.n_local, grasp.in_plane_axis, grasp.face_index), "phase de grip")) return 1;

    // Attacher l’objet
    //std::string hand_link = "panda_hand";
    //std::vector<std::string> touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
    manager.attachObject(object_id);

    // Déplacement vers le goal
    auto solver_goal = loadSolver("/goal_solver"); 
    SolverType solver = solverFromString(solver_goal);
    auto goal_pose = loadObjectPose("/goal");
    if (!check(manager.moveToGoal(goal_pose, solver_goal), "phase déplacement vers l'objectif")) return 1;

    // Ouvrir la pince pour déposer l'objet
    manager.openGripper();

    // Détacher l’objet
    manager.detachObject(object_id);

    ros::shutdown();
}