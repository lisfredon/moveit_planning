#include "moveit_planning/gripper_controller.h"
#include "moveit_planning/robot_utils.h"
#include <ros/ros.h>

GripperController::GripperController(const std::string& gripper_group_name)
    : gripper_group_(gripper_group_name) {}

void GripperController::open()
{
    ::openGripper(gripper_group_);
}

void GripperController::close(double target)
{
    ::closeGripper(gripper_group_, target);
}

moveit::planning_interface::MoveGroupInterface& GripperController::getMoveGroup()
{
    return gripper_group_;
}
