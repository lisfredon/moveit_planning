#include "moveit_planning/pick_place_manager.h"
#include "moveit_planning/solvers_utils.h"
#include "moveit_planning/visualization_utils.h"
#include "moveit_planning/grasp_utils.h"
#include "moveit_planning/load_add_object.h"
#include "moveit_planning/robot_utils.h"
#include "moveit_planning/moveTo_utils.h"
#include "moveit_planning/error_handling.h"
#include "moveit_planning/pick_place_manager.h"

PickPlaceManager::PickPlaceManager(const std::string& arm_group_name,
                                   const std::string& gripper_group_name)
    : arm_group_(arm_group_name),
      gripper_group_(gripper_group_name)
{
    ROS_INFO("PickPlaceManager initialisé");
}

void PickPlaceManager::openGripper() {
    ::openGripper(gripper_group_);
}

void PickPlaceManager::closeGripper(double target) {
    ::closeGripper(gripper_group_, target);
}

moveit_msgs::CollisionObject PickPlaceManager::addObject(
    const std::string& object_id,
    const geometry_msgs::Pose& pose,
    const std::vector<double>& size) 
{
    return addObjectToScene(scene_, object_id, pose, size, arm_group_.getPlanningFrame());
}

bool PickPlaceManager::approach(const geometry_msgs::Pose& obj_pose,
                                const tf2::Vector3& n_local,
                                const tf2::Vector3& in_plane_axis) 
{
    return ::approch(arm_group_, gripper_group_, obj_pose, n_local, in_plane_axis);
}

bool PickPlaceManager::grip(const geometry_msgs::Pose& obj_pose,
                            const std::vector<double>& obj_size,
                            const tf2::Vector3& n_local,
                            const tf2::Vector3& in_plane_axis,
                            int face_index) 
{
    return ::grip(arm_group_, gripper_group_, obj_pose, obj_size, n_local, in_plane_axis, face_index);
}

void PickPlaceManager::attachObject(const std::string& object_id) {
    std::vector<std::string> touch_links = getGroupLinks(gripper_group_.getName());

    // par défaut, on prend le "end effector link" du groupe
    std::string hand_link = gripper_group_.getEndEffectorLink();
    if (hand_link.empty() && !touch_links.empty()) {
        hand_link = touch_links.front();
    }

    ::attachObject(arm_group_, object_id, hand_link, touch_links);
}


void PickPlaceManager::detachObject(const std::string& object_id) {
    ::detachObject(scene_, arm_group_, object_id);
}

bool PickPlaceManager::moveToGoal(const geometry_msgs::Pose& goal_pose,
                                  const std::string& solver_name) 
{
    SolverType solver = solverFromString(solver_name);
    return ::moveTo(arm_group_, goal_pose, solver, "déplacement vers l'objectif");
}
