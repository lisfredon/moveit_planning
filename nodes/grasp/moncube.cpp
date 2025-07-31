#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "attraper_cube");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setPoseReferenceFrame("world");

  // 1. Définir et ajouter le cube (identique à avant)
  moveit_msgs::CollisionObject cube;
  cube.header.frame_id = "world";
  cube.id = "box1";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.05, 0.05, 0.05};

  geometry_msgs::Pose cube_pose;
  cube_pose.orientation.w = 1;
  cube_pose.position.x = 0.5;
  cube_pose.position.y = 0.0;
  cube_pose.position.z = 0.5;

  cube.primitives.push_back(primitive);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;

  planning_scene_interface.applyCollisionObjects({cube});

  // 2. Approcher juste en dessous du cube
geometry_msgs::Pose pose_approche = cube_pose;
pose_approche.position.z += 0.10; // au-dessus du cube

tf2::Quaternion q;
q.setRPY( 0.00001, 0, 0); 
pose_approche.orientation = tf2::toMsg(q);

move_group.setStartStateToCurrentState();
move_group.setGoalOrientationTolerance(0.1);
move_group.setPoseTarget(pose_approche);

  geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
ROS_INFO_STREAM("Pose actuelle: " << current_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan_approche;
  if (move_group.plan(plan_approche) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan_approche);
    ros::Duration(0.5).sleep();  // ← petit délai pour stabiliser
    move_group.setStartStateToCurrentState(); // puis re-synchronise
    ROS_INFO("Approche réussie");
  }
  else
  {
    ROS_WARN("Échec de l’approche");
    return 1;
  }

  // 3. Descendre pour saisir le cube (directement sur le cube)
  geometry_msgs::Pose pose_saisie = cube_pose;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pose_saisie);
  moveit::planning_interface::MoveGroupInterface::Plan plan_saisie;
  if (move_group.plan(plan_saisie) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan_saisie);
    ros::Duration(0.5).sleep();  // ← petit délai pour stabiliser
    move_group.setStartStateToCurrentState(); // puis re-synchronise
    ROS_INFO("Position de saisie atteinte");
  }
  else
  {
    ROS_WARN("Échec de la position de saisie");
    return 1;
  }

  // 4. Simuler la fermeture de la pince 
  ROS_INFO("Simuler fermeture pince ");

  // 5. Attacher l’objet au robot
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand"; 
  attached_object.object = cube;
  attached_object.object.operation = attached_object.object.ADD;

  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  ROS_INFO("Objet attaché au robot");

  // 6. Lever le bras avec l’objet attaché
  geometry_msgs::Pose pose_levage = pose_saisie;
  pose_levage.position.z += 0.2; 

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pose_levage);
  moveit::planning_interface::MoveGroupInterface::Plan plan_levage;
  if (move_group.plan(plan_levage) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(plan_levage);
    ros::Duration(0.5).sleep();  // ← petit délai pour stabiliser
    move_group.setStartStateToCurrentState(); // puis re-synchronise
    ROS_INFO("Levage réussi avec l’objet");
  }
  else
  {
    ROS_WARN("Échec du levage");
    return 1;
  }

  ros::shutdown();
  return 0;
}