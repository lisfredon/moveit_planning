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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); spinner.start();

    // Charger le robot choisi
    PickPlaceManager manager = initRobot(nh);
    manager.openGripper();

    // Charger la liste des cubes
    std::vector<std::string> cube_names;
    if (!nh.getParam("/cubes", cube_names)) {
        ROS_ERROR("Impossible de charger la liste des cubes (/cubes)");
        return 1;
    }

    // Ajouter tous les cubes dans la scène
    for (const auto& cube_ns : cube_names) {
        manager.addObject(loadObjectID("/" + cube_ns + "/id"),
                          loadObjectPose("/" + cube_ns),
                          loadObjectSize("/" + cube_ns + "/size"));
    }

    // Cube cible
    std::string target_cube; nh.param<std::string>("/target_cube", target_cube, "cube1");
    geometry_msgs::Pose goal_pose = loadObjectPose("/goal");

    // Grasp
    auto obj_size = loadObjectSize("/" + target_cube + "/size");
    auto obj_pose = loadObjectPose("/" + target_cube);

    // Choisir la face pour le grasp
    GraspChoice grasp;
    if (!check(chooseGraspFace(nh, manager.getGripperGroup(), obj_size, grasp), "choix de la face de grasp")) return 1;

    //Visualisation des axes de l'objet pour debug 
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(target_cube, 1, true);
    visualizeCubeFaces(marker_pub, obj_pose);

    //phase de pick
    if (!manager.pick(target_cube, obj_pose, obj_size, grasp.n_local, grasp.in_plane_axis, grasp.face_index)) return 1;

    auto solver_goal = loadSolver("/goal_solver"); 

    //phase de place 
    if (!manager.place(target_cube, obj_pose, goal_pose, solver_goal)) return 1;

    ROS_INFO("Pick & Place terminé avec succès !");
    ros::shutdown();
    return 0;
}