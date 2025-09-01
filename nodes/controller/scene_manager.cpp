#include "moveit_planning/scene_manager.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/robot_utils.h"
#include <ros/ros.h>

SceneManager::SceneManager() {}

void SceneManager::addObject(const std::string& object_id,
                             const geometry_msgs::Pose& pose,
                             const std::vector<double>& size,
                             const std::string& reference_frame)
{
    ::addObjectToScene(scene_, object_id, pose, size, reference_frame);
}

void SceneManager::attachObject(const std::string& object_id,
                                moveit::planning_interface::MoveGroupInterface& arm_group,
                                moveit::planning_interface::MoveGroupInterface& gripper_group)
{
    std::vector<std::string> touch_links = getGroupLinks(gripper_group.getName());
    std::string hand_link = gripper_group.getEndEffectorLink();
    if (hand_link.empty() && !touch_links.empty())
        hand_link = touch_links.front();

    ::attachObject(arm_group, object_id, hand_link, touch_links);
}

void SceneManager::detachObject(const std::string& object_id,
                                moveit::planning_interface::MoveGroupInterface& arm_group)
{
    ::detachObject(scene_, arm_group, object_id);
}
