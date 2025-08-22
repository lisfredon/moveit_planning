#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/moveTo_utils.h"
#include "moveit_planning/robot_utils.h"

class PickPlaceManager {
public:
    PickPlaceManager(const std::string& arm_group_name,
                     const std::string& gripper_group_name);

    void openGripper();
    void closeGripper(double target);

    moveit_msgs::CollisionObject addObject(const std::string& object_id,
                                           const geometry_msgs::Pose& pose,
                                           const std::vector<double>& size);

    bool approach(const geometry_msgs::Pose& obj_pose,
                  const tf2::Vector3& n_local,
                  const tf2::Vector3& in_plane_axis);

    bool grip(const geometry_msgs::Pose& obj_pose,
              const std::vector<double>& obj_size,
              const tf2::Vector3& n_local,
              const tf2::Vector3& in_plane_axis,
              int face_index);

    void attachObject(const std::string& object_id);
    void detachObject(const std::string& object_id);

    bool moveToGoal(const geometry_msgs::Pose& goal_pose,
                    const std::string& solver_name);

    moveit::planning_interface::MoveGroupInterface& getGripperGroup() { return gripper_group_; }

private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    moveit::planning_interface::PlanningSceneInterface scene_;
};
