#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>
#include <vector>
#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>

geometry_msgs::Pose generateGraspPose(
    const geometry_msgs::Pose& cube_pose,
    const tf2::Vector3& n_local,
    double offset = 0.00)
{
    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose.orientation, q_cube);
    tf2::Matrix3x3 R_cube(q_cube);

    tf2::Vector3 n_global = R_cube * n_local;
    tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
    tf2::Vector3 p_gripper = p_cube - offset * n_global;

    tf2::Vector3 y_global = R_cube * tf2::Vector3(0, 1, 0);
    tf2::Vector3 z_global = R_cube * tf2::Vector3(0, 0, 1);

    tf2::Vector3 z_axis = n_global.normalized();
    tf2::Vector3 x_axis = y_global.normalized();
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();

    tf2::Matrix3x3 R_gripper(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());

    tf2::Quaternion q_gripper;
    R_gripper.getRotation(q_gripper);

    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = p_gripper.x();
    grasp_pose.position.y = p_gripper.y();
    grasp_pose.position.z = p_gripper.z();
    grasp_pose.orientation = tf2::toMsg(q_gripper);

    return grasp_pose;
}

// Charger pose depuis paramètre
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

    // Charger cube
    std::vector<double> cube_size;
    ros::param::get("/cube/size", cube_size);
    geometry_msgs::Pose object_pose = loadParam("/cube");

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

    std::vector<tf2::Vector3> face_normals = {
        tf2::Vector3(1, 0, 0),  tf2::Vector3(-1, 0, 0),
        tf2::Vector3(0, 1, 0),  tf2::Vector3(0, -1, 0),
        tf2::Vector3(0, 0, 1),  tf2::Vector3(0, 0, -1)
    };

    double approach_offset = 0.10; // 10 cm en retrait
    double grip_offset = 0.00;     // directement sur la face

    for (size_t i = 0; i < face_normals.size(); ++i) {
        // Phase d'approche
        geometry_msgs::Pose approach_pose = generateGraspPose(object_pose, face_normals[i], approach_offset);
        move_group.setPoseTarget(approach_pose);
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        if (move_group.plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Plan approche trouvé pour face " << i);
            move_group.execute(approach_plan);
        }

        // Phase de grip
        geometry_msgs::Pose grip_pose = generateGraspPose(object_pose, face_normals[i], grip_offset);
        move_group.setPoseTarget(grip_pose);
        moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
        if (move_group.plan(grip_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO_STREAM("Plan grip trouvé pour face " << i);
            move_group.execute(grip_plan);

            // Fermer la pince
            gripper_group.setJointValueTarget(std::vector<double>{0.0, 0.0});
            gripper_group.move();

            // Attacher cube
            moveit_msgs::AttachedCollisionObject attached_object;
            attached_object.link_name = move_group.getEndEffectorLink();
            attached_object.object = cube;
            attached_object.object.operation = attached_object.object.ADD;
            planning_scene_interface.applyAttachedCollisionObject(attached_object);
        }
    }

    ros::shutdown();
}
