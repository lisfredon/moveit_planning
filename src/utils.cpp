#include "moveit_planning/utils.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cmath>

// Création de la pose de préhension selon la direction
geometry_msgs::Pose createGraspPose(const std::string& direction, const geometry_msgs::Pose& object_pose) {
  geometry_msgs::Pose pose = object_pose;
  tf2::Quaternion orientation;

  if (direction == "F_verticale") {
    orientation.setRPY(-M_PI / 2, 0.0, -M_PI / 2);  
    pose.position.z -= 0.10;
  } else if (direction == "F_horizontale") {//haut perpendiculaire
    orientation.setRPY(0.0, M_PI / 2, 0.0); // pince horizontale
    pose.position.z -= 0.10;
  } else if (direction == "F_Droite") {
    orientation.setRPY(0.0, -M_PI / 2, -M_PI / 2);  // prise par le côté
    pose.position.z -= 0.10;
    //pose.position.y -= 0.10;
  } else if (direction == "F_Gauche") {
    orientation.setRPY(0.0, -M_PI / 2, M_PI / 2);
    pose.position.z -= 0.10;
  } else if (direction == "H_perpendiculaire") {//haut perpendiculaire
    orientation.setRPY(-M_PI, 0.0, -M_PI / 2);
    pose.position.z -= 0.10;
  } else if (direction == "H_parallele") {//haut parallele
    orientation.setRPY(-M_PI, 0.0, 0.0);  // pince parallèle à l'axe X (horizontal)
    pose.position.z -= 0.10;
  } else {
    //haut parallele
    ROS_WARN("Direction inconnue");
  }

  orientation.normalize();
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}