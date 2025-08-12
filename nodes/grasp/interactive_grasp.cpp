#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/Constraints.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <cmath>

#include "moveit_planning/utils.h"

/*
// Pose de l'objet dans le repère panda_link0
geometry_msgs::Pose getObjectPose() {
  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}
  */


int main(int argc, char** argv) {
  ros::init(argc, argv, "interactive_grasp");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");

  // Paramètres de planification
  move_group.setPlanningTime(20.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setPoseReferenceFrame("panda_link0");

  // Lire la position de l'objet depuis les paramètres ROS
  double ox, oy, oz;
  if (!nh.getParam("/grasp/obj_pos_x", ox) ||
      !nh.getParam("/grasp/obj_pos_y", oy) ||
      !nh.getParam("/grasp/obj_pos_z", oz)) {
      ROS_ERROR("Impossible de récupérer la position de l'objet depuis les paramètres ROS.");
      return 1;
  }

  geometry_msgs::Pose object_pose;
  object_pose.position.x = ox;
  object_pose.position.y = oy;
  object_pose.position.z = oz;
  object_pose.orientation.w = 1.0;


  // Lecture de la direction
  std::string direction;
  std::cout << "Comment veux-tu saisir l'objet ? (F_verticale / F_horizontale / F_Droite / F_Gauche / H_perpendiculaire / H_parallele ) : ";
  std::cin >> direction;

  // Pose cible
  //geometry_msgs::Pose object_pose = getObjectPose();

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
    new planning_scene_monitor::PlanningSceneMonitor("robot_description")
  );

  planning_scene_monitor->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);

  bool in_collision = scene->isStateColliding(*move_group.getCurrentState(), "panda_manipulator");

  if (in_collision) {
    ROS_WARN("Le robot est en collision dans son état **initial** !");
  } else {
    ROS_INFO("Le robot est dans un état sans collision.");
  }

  geometry_msgs::Pose target_pose = createGraspPose(direction, object_pose);

  ROS_INFO_STREAM("Pose de préhension cible :");
  ROS_INFO_STREAM("Position : [" << target_pose.position.x << ", "
                                 << target_pose.position.y << ", "
                                 << target_pose.position.z << "]");
  ROS_INFO_STREAM("Orientation (quaternion) : ["
                << target_pose.orientation.x << ", "
                << target_pose.orientation.y << ", "
                << target_pose.orientation.z << ", "
                << target_pose.orientation.w << "]");

  ROS_INFO_STREAM("Orientation de l'objet (quaternion) : ["
                << object_pose.orientation.x << ", "
                << object_pose.orientation.y << ", "
                << object_pose.orientation.z << ", "
                << object_pose.orientation.w << "]");


  // Etape 1 : Mouvement d'approche sans contrainte
  geometry_msgs::Pose approach_pose = target_pose;
  approach_pose.position.z += 0.1; // approche verticale par le haut

  move_group.clearPathConstraints();
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(approach_pose);

  moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
  robot_state::RobotStatePtr current_state = move_group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup("panda_manipulator");

  bool found_ik = current_state->setFromIK(joint_model_group, approach_pose, 5.0);
  if (!found_ik) {
    ROS_ERROR("Aucune solution IK trouvée pour la pose d'approche !");
    return 1;
  }

  if (move_group.plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Plan d'approche sans contrainte trouvé. Exécution...");
    move_group.execute(approach_plan);
  } else {
    ROS_ERROR("Échec du plan d'approche sans contrainte.");
    std::vector<std::string> links_in_collision;
    scene->getCollidingLinks(links_in_collision, *move_group.getCurrentState());

    for (const auto& link : links_in_collision) {
      ROS_WARN_STREAM("Lien en collision : " << link);
    }

    return 1; // stop le programme si cette phase échoue
  }

  moveit_msgs::Constraints path_constraints;
  move_group.setStartStateToCurrentState();
  move_group.setPathConstraints(path_constraints);
  move_group.setPoseTarget(target_pose);

  // Etape 2 : Attacher l'objet à la pince pour simuler la saisie
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::string object_id = "cube"; 
  std::string link_name = "panda_hand";  

  move_group.attachObject(object_id, link_name);

  ROS_INFO("Objet attaché à la pince.");
  nh.setParam("/grasp/last_grasp_pose/position/x", target_pose.position.x);
  nh.setParam("/grasp/last_grasp_pose/position/y", target_pose.position.y);
  nh.setParam("/grasp/last_grasp_pose/position/z", target_pose.position.z);
  nh.setParam("/grasp/last_grasp_pose/orientation/x", target_pose.orientation.x);
  nh.setParam("/grasp/last_grasp_pose/orientation/y", target_pose.orientation.y);
  nh.setParam("/grasp/last_grasp_pose/orientation/z", target_pose.orientation.z);
  nh.setParam("/grasp/last_grasp_pose/orientation/w", target_pose.orientation.w);

  ros::shutdown();
  return 0;
}
