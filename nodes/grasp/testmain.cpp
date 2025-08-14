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
#include "moveit_planning/load_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/add_object.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Charger cube
    std::vector<double> cube_size = loadCubeSizeParam("/cube/size");
    geometry_msgs::Pose cube_pose = loadParam("/cube");

    // Ajouter cube à MoveIt
    addCubeToScene(planning_scene_interface, "cube", cube_pose, cube_size);

    // Normales locales des faces du cube
    std::vector<tf2::Vector3> face_normals = {
        tf2::Vector3(1,0,0),  tf2::Vector3(-1,0,0),
        tf2::Vector3(0,1,0),  tf2::Vector3(0,-1,0),
        tf2::Vector3(0,0,1),  tf2::Vector3(0,0,-1)
    };

    // Axes "largeur"
    std::vector<tf2::Vector3> in_plane_axes_width = {
        tf2::Vector3(0,0,1),  // +X
        tf2::Vector3(0,0,1),  // -X
        tf2::Vector3(1,0,0),  // +Y
        tf2::Vector3(1,0,0),  // -Y
        tf2::Vector3(1,0,0),  // +Z
        tf2::Vector3(1,0,0)   // -Z
    };

    // Axes "longueur" (orthogonal à largeur et normale)
    std::vector<tf2::Vector3> in_plane_axes_length = {
        tf2::Vector3(0,1,0),  // +X
        tf2::Vector3(0,1,0),  // -X
        tf2::Vector3(0,0,1),  // +Y
        tf2::Vector3(0,0,1),  // -Y
        tf2::Vector3(0,1,0),  // +Z
        tf2::Vector3(0,1,0)   // -Z
    };

    // Choix de la face et de l'orientation dans le plan
    int face_index = 5; // 0:+X, 1:-X, 2:+Y, ...
    bool use_width = false; // true = "largeur", false = "longueur"
    tf2::Vector3 n_local = face_normals[face_index];
    tf2::Vector3 in_plane_axis = use_width ? 
        in_plane_axes_width[face_index] : 
        in_plane_axes_length[face_index];

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cube", 1, true);

    std::vector<std::string> face_names = {"+X","-X","+Y","-Y","+Z","-Z"};
    publishFaceNormalsWithText(cube_pose, face_normals, face_names, marker_pub);

    // Liste des solveurs à tester
    std::vector<std::pair<SolverType, std::string>> solvers = {
        {SolverType::CARTESIAN, "CARTESIAN"},
        {SolverType::OMPL, "OMPL"},
        {SolverType::JOINT_INTERPOLATION, "JOINT_INTERPOLATION"}
    };

    // Phase d'approche
    double approach_offset = 0.05; // 5 cm devant la face
    geometry_msgs::Pose approach_pose = generateGraspPose(cube_pose, n_local, in_plane_axis, approach_offset);

    if (!planAndExecute(move_group, approach_pose, SolverType::OMPL)) {
        ROS_ERROR("Impossible de planifier la phase d'approche !");
        return 1;
    }
    else ROS_INFO("Phase d'approche effectuée.");

    // Phase de grip
    double grip_offset = 0.0;
    geometry_msgs::Pose grip_pose = generateGraspPose(cube_pose, n_local, in_plane_axis, grip_offset);

    if (!planAndExecute(move_group, grip_pose, SolverType::OMPL)) {
        ROS_ERROR("Impossible de planifier la phase de grip !");
        return 1;
    }
    else ROS_INFO("Phase de grip effectuée.");

    ros::shutdown();
}