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
#include "moveit_planning/move_to_goal.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Charger objet
    auto objet_size = loadObjectSize("/cube/size");
    auto objet_pose = loadObjectPose("/cube");
    auto object_id = loadObjectID("/cube/id");

    // Ajouter objet à MoveIt
    auto objet = addObjectToScene(planning_scene_interface, object_id, objet_pose, objet_size, move_group.getPlanningFrame());

    // Choix de la face et de l'orientation dans le plan
    auto face_index = loadGoalFace("/how_take_cube/choice_face");
    tf2::Vector3 n_local = getNormalObject(face_index);
    auto side_face = loadSideFace("/how_take_cube/side_face");
    tf2::Vector3 in_plane_axis = getObjectAxis(face_index, side_face);
    
    //Visualisation des axes de l'objet pour debug 
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(object_id, 1, true);
    visualizeCubeFaces(marker_pub, objet_pose);

    // Phase d'approche
    if (!moveToGraspPhase(move_group, objet_pose, n_local, in_plane_axis, 0.05, SolverType::OMPL, "approche")) return 1;
    
    //phase de grip
    if (!grip(move_group, gripper_group, objet_pose, objet_size, n_local, in_plane_axis, face_index)) {
        ROS_ERROR("Échec de la préhension");
        return 1;
    }

    // Attacher l’objet
    std::string hand_link = "panda_hand";
    std::vector<std::string> touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
    attachObject(move_group, object_id, hand_link, touch_links);


    // Déplacement vers le goal
    auto goal_pose = loadObjectPose("/goal");
    if (!moveToGoal(move_group, goal_pose)) {
        ROS_ERROR("Échec du déplacement vers le goal !");
        return 1;
    }

    // Ouvrir la pince pour déposer l'objet
    openGripper(gripper_group);

    // Détacher l’objet
    detachObject(planning_scene_interface, move_group, object_id);

    ros::shutdown();
}