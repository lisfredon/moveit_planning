
#include "moveit_planning/pick_place_manager.h"
#include <ros/ros.h>

PickPlaceManager::PickPlaceManager(const std::string& arm_group_name,
                                   const std::string& gripper_group_name)
    : arm_(arm_group_name),
      gripper_(gripper_group_name),
      scene_() {}

void PickPlaceManager::openGripper()
{
    gripper_.open();
}

void PickPlaceManager::closeGripper(double target)
{
    gripper_.close(target);
}

void PickPlaceManager::addObject(const std::string& object_id,
                                 const geometry_msgs::Pose& pose,
                                 const std::vector<double>& size)
{
    scene_.addObject(object_id, pose, size, arm_.getMoveGroup().getPlanningFrame());
}

bool PickPlaceManager::pick(const std::string& object_id,
                            const geometry_msgs::Pose& obj_pose,
                            const std::vector<double>& obj_size,
                            const tf2::Vector3& n_local,
                            const tf2::Vector3& in_plane_axis,
                            int face_index)
{
    if (!arm_.approachObject(gripper_.getMoveGroup(), obj_pose, n_local, in_plane_axis))
        return false;

    if (!arm_.gripObject(gripper_.getMoveGroup(), obj_pose, obj_size, n_local, in_plane_axis, face_index))
        return false;


    scene_.attachObject(object_id, arm_.getMoveGroup(), gripper_.getMoveGroup());
    return true;
}

bool PickPlaceManager::place(const std::string& object_id,
                            const geometry_msgs::Pose& object_pose,
                            geometry_msgs::Pose goal_pose,
                            SolverType solver_name)
{
    // Calcul de la rotation relative
    tf2::Quaternion q_eef, q_obj;
    tf2::fromMsg(arm_.getMoveGroup().getCurrentPose().pose.orientation, q_eef);
    tf2::fromMsg(object_pose.orientation, q_obj);
    tf2::Quaternion q_rel = q_eef.inverse() * q_obj;

    tf2::Quaternion q_goal;
    tf2::fromMsg(goal_pose.orientation, q_goal);
    q_goal.normalize();
    goal_pose.orientation = tf2::toMsg(q_goal * q_rel.inverse());

    if (!arm_.moveToPose(goal_pose, solver_name))
        return false;

    gripper_.open();
    scene_.detachObject(object_id, arm_.getMoveGroup());
    return true;
}

moveit::planning_interface::MoveGroupInterface& PickPlaceManager::getArmGroup()
{
    return arm_.getMoveGroup();
}

moveit::planning_interface::MoveGroupInterface& PickPlaceManager::getGripperGroup()
{
    return gripper_.getMoveGroup();
}
