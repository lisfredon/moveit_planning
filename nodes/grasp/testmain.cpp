#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/SolidPrimitive.h>
#include <vector>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setPoseReferenceFrame("panda_link0");

    // Lire position/orientation cube
    double ox, oy, oz, px, py, pz, pw;
    nh.getParam("/grasp/obj_pos_x", ox);
    nh.getParam("/grasp/obj_pos_y", oy);
    nh.getParam("/grasp/obj_pos_z", oz);
    nh.getParam("/grasp/obj_ori_x", px);
    nh.getParam("/grasp/obj_ori_y", py);
    nh.getParam("/grasp/obj_ori_z", pz);
    nh.getParam("/grasp/obj_ori_w", pw);

    geometry_msgs::Pose cube_pose;
    cube_pose.position.x = ox;
    cube_pose.position.y = oy;
    cube_pose.position.z = oz;
    //cube_pose.orientation.x = 0.7;
    cube_pose.orientation.y = py;
    cube_pose.orientation.z = pz;
    cube_pose.orientation.w = pw;

    // Ajouter cube à la scène
    moveit_msgs::CollisionObject cube;
    cube.id = "cube";
    cube.header.frame_id = move_group.getPlanningFrame();
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.05, 0.05, 0.05};
    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;
    planning_scene_interface.applyCollisionObject(cube);

    // Calcul orientation pince
    tf2::Vector3 n_local(1, 0, 0);
    tf2::Quaternion q_cube;
    tf2::fromMsg(cube_pose.orientation, q_cube);
    tf2::Matrix3x3 R_cube(q_cube);
    tf2::Vector3 n_global = R_cube * n_local;
    double d = 0.0;
    tf2::Vector3 p_cube(cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
    tf2::Vector3 p_gripper = p_cube - d * n_global;

    tf2::Vector3 z_axis = n_global.normalized();
    tf2::Vector3 arbitrary(0, 0, 1);
    if (fabs(z_axis.dot(arbitrary)) > 0.9)
        arbitrary = tf2::Vector3(0, 1, 0);
    tf2::Vector3 x_axis = arbitrary.cross(z_axis).normalized();
    tf2::Vector3 y_axis = z_axis.cross(x_axis);
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

    ROS_INFO_STREAM("Grasp pose: " << grasp_pose);

    move_group.setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        gripper_group.setJointValueTarget(std::vector<double>{0.0, 0.0});
        gripper_group.move();

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = move_group.getEndEffectorLink();
        attached_object.object = cube;
        attached_object.object.operation = attached_object.object.ADD;
        planning_scene_interface.applyAttachedCollisionObject(attached_object);
    }

    ros::shutdown();
}
