#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spawn_object");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Créer un objet cube
  moveit_msgs::CollisionObject object;
  object.id = "cube";
  object.header.frame_id = "panda_link0";  // ou "world" selon ton cas

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.05, 0.05, 0.05}; // taille 5cm

  geometry_msgs::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.5;
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  planning_scene_interface.applyCollisionObject(object);

  ROS_INFO("Objet ajouté sans bouger le bras.");

  ros::waitForShutdown();
  return 0;
}
