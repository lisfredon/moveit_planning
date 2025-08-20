#include "moveit_planning/approch_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/solvers_utils.h"

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

void publishFaceNormalsWithText(const geometry_msgs::Pose& cube_pose,
                                const std::vector<tf2::Vector3>& face_normals,
                                const std::vector<std::string>& face_names,
                                ros::Publisher& marker_pub)
{
    visualization_msgs::MarkerArray marker_array;

    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose.orientation, q_cube);
    tf2::Matrix3x3 R_cube(q_cube);
    tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);

    for (size_t i = 0; i < face_normals.size(); ++i) {
        tf2::Vector3 n_global = R_cube * face_normals[i];

        // --- Flèche ---
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "panda_link0";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "face_normals";
        arrow.id = i;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start, end;
        start.x = p_cube.x();
        start.y = p_cube.y();
        start.z = p_cube.z();
        end.x = p_cube.x() + 0.1 * n_global.x();
        end.y = p_cube.y() + 0.1 * n_global.y();
        end.z = p_cube.z() + 0.1 * n_global.z();
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.01;
        arrow.scale.y = 0.02;
        arrow.scale.z = 0.0;

        // Couleur flèche
        if (face_names[i] == "+X") { arrow.color.r = 1.0; arrow.color.g = 0; arrow.color.b = 0; }
        else if (face_names[i] == "-X") { arrow.color.r = 0.5; arrow.color.g = 0; arrow.color.b = 0; }
        else if (face_names[i] == "+Y") { arrow.color.r = 0; arrow.color.g = 1.0; arrow.color.b = 0; }
        else if (face_names[i] == "-Y") { arrow.color.r = 0; arrow.color.g = 0.5; arrow.color.b = 0; }
        else if (face_names[i] == "+Z") { arrow.color.r = 0; arrow.color.g = 0; arrow.color.b = 1.0; }
        else if (face_names[i] == "-Z") { arrow.color.r = 0; arrow.color.g = 0; arrow.color.b = 0.5; }
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(0);
        marker_array.markers.push_back(arrow);

        // --- Texte ---
        visualization_msgs::Marker text;
        text.header.frame_id = "panda_link0";
        text.header.stamp = ros::Time::now();
        text.ns = "face_normals_text";
        text.id = i + 100; // ID différent pour ne pas confliter avec les flèches
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;

        text.pose.position.x = end.x;
        text.pose.position.y = end.y;
        text.pose.position.z = end.z;
        text.pose.orientation.w = 1.0;

        text.scale.z = 0.05;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;

        text.text = face_names[i];
        text.lifetime = ros::Duration(0);
        marker_array.markers.push_back(text);
    }

    marker_pub.publish(marker_array);
}

// Charger une pose depuis le paramètre ROS
geometry_msgs::Pose loadParam(const std::string& param_namespace) {
    std::vector<double> pos, ori;
    geometry_msgs::Pose pose;
    if (!ros::param::get(param_namespace + "/position", pos) ||
        !ros::param::get(param_namespace + "/orientation", ori)) {
        ROS_ERROR_STREAM("Impossible de charger les paramètres pour " << param_namespace);
        throw std::runtime_error("Paramètres manquants");
    }
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];
    return pose;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Charger cube et normales...
    std::vector<double> cube_size;
    ros::param::get("/cube/size", cube_size);
    geometry_msgs::Pose cube_pose = loadParam("/cube");

    moveit_msgs::CollisionObject cube;
    cube.header.frame_id = "panda_link0";
    cube.id = "cube";
    shape_msgs::SolidPrimitive cube_primitive;
    cube_primitive.type = shape_msgs::SolidPrimitive::BOX;
    cube_primitive.dimensions = cube_size;
    cube.primitives.push_back(cube_primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyCollisionObjects({cube});

    std::vector<tf2::Vector3> face_normals = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };
    std::vector<std::string> face_names = {"+X","-X","+Y","-Y","+Z","-Z"};
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cube_face_normals", 1, true);
    publishFaceNormalsWithText(cube_pose, face_normals, face_names, marker_pub);

    int face_index = 0;
    tf2::Vector3 n_local = face_normals[face_index];
    tf2::Vector3 in_plane_axis(0,0,1);

    // Phase d'approche
    double approach_offset = 0.05;
    geometry_msgs::Pose approach_pose = generateGraspPose(cube_pose, n_local, in_plane_axis, approach_offset);

    // Liste des solveurs à tester
    std::vector<std::pair<SolverType, std::string>> solvers = {
        {SolverType::CARTESIAN, "CARTESIAN"},
        {SolverType::OMPL, "OMPL"},
        {SolverType::JOINT_INTERPOLATION, "JOINT_INTERPOLATION"}
    };

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

    // Fermer la pince
    gripper_group.setJointValueTarget(std::vector<double>{0.0, 0.0});
    gripper_group.move();

    // Attacher l'objet
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = move_group.getEndEffectorLink();
    attached_object.object = cube;
    attached_object.object.operation = attached_object.object.ADD;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    
    ros::shutdown();
}