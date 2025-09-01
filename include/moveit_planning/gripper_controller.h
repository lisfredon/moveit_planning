#pragma once
#include <moveit/move_group_interface/move_group_interface.h>

class GripperController {
public:
    GripperController(const std::string& gripper_group_name);

    void open();
    void close(double target = 0.0);

    moveit::planning_interface::MoveGroupInterface& getMoveGroup();

private:
    moveit::planning_interface::MoveGroupInterface gripper_group_;
};
