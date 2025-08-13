#include "moveit_planning/approch_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/transport_utils.h"
#include "moveit_planning/retreat_utils.h"
#include "moveit_planning/solvers_utils.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <iostream>
#include <string>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <vector>

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

// Charger poses et tailles
    std::vector<double> cube_size;
        if (!ros::param::get("/cube/size", cube_size) || cube_size.size() != 3) {
            ROS_ERROR("Impossible de charger la taille du cube !");
            return 1;
        }
        geometry_msgs::Pose object_pose = loadParam("/cube");

        std::vector<double> table_size;
        if (!ros::param::get("/table/size", table_size) || table_size.size() != 3) {
            ROS_ERROR("Impossible de charger la taille de la table !");
            return 1;
        }
        geometry_msgs::Pose table_pose = loadParam("/table");

        geometry_msgs::Pose goal_pose = loadParam("/goal");

        // Ajouter cube
        moveit_msgs::CollisionObject cube;
        cube.header.frame_id = "panda_link0";
        cube.id = "cube";

        shape_msgs::SolidPrimitive cube_primitive;
        cube_primitive.type = shape_msgs::SolidPrimitive::BOX;
        cube_primitive.dimensions = cube_size;

        cube.primitives.push_back(cube_primitive);
        cube.primitive_poses.push_back(object_pose);
        cube.operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects({cube});

        /*
        // Ajouter table
        moveit_msgs::CollisionObject table;
        table.header.frame_id = "panda_link0";
        table.id = "table";

        shape_msgs::SolidPrimitive table_primitive;
        table_primitive.type = shape_msgs::SolidPrimitive::BOX;
        table_primitive.dimensions = table_size;

        table.primitives.push_back(table_primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::CollisionObject::ADD;

        planning_scene_interface.applyCollisionObjects({table});
    
        move_group.setPlanningTime(10.0); 
        */

    std::vector<tf2::Vector3> face_normals = {
        tf2::Vector3(1, 0, 0),  // +X
        tf2::Vector3(-1, 0, 0), // -X
        tf2::Vector3(0, 1, 0),  // +Y
        tf2::Vector3(0, -1, 0), // -Y
        tf2::Vector3(0, 0, 1),  // +Z
        tf2::Vector3(0, 0, -1)  // -Z
    };

    for (size_t i = 0; i < face_normals.size(); ++i) {
        geometry_msgs::Pose grasp_pose = generateAlignedGraspPose(object_pose, face_normals[i], 0.00);
        ROS_INFO_STREAM("Test face " << i << " : " << grasp_pose);

        move_group.setPoseTarget(grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Plan trouvé pour face " << i);
            move_group.execute(plan);
            gripper_group.setJointValueTarget(std::vector<double>{0.0, 0.0});
            gripper_group.move();

            // Attacher l'objet à l'effecteur
            moveit_msgs::AttachedCollisionObject attached_object;
            attached_object.link_name = move_group.getEndEffectorLink();
            attached_object.object = cube;
            attached_object.object.operation = attached_object.object.ADD;
            planning_scene_interface.applyAttachedCollisionObject(attached_object);
        }
    }

    ros::shutdown();
}