#pragma once
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>

class SceneManager {
public:
    SceneManager();

    void addObject(const std::string& object_id,
                   const geometry_msgs::Pose& pose,
                   const std::vector<double>& size,
                   const std::string& reference_frame);

    void attachObject(const std::string& object_id,
                      moveit::planning_interface::MoveGroupInterface& arm_group,
                      moveit::planning_interface::MoveGroupInterface& gripper_group);

    void detachObject(const std::string& object_id,
                      moveit::planning_interface::MoveGroupInterface& arm_group);

private:
    moveit::planning_interface::PlanningSceneInterface scene_;
};
