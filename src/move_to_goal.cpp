#include "moveit_planning/move_to_goal.h"

bool moveToGoal(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& goal_pose) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setPoseTarget(goal_pose);
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        return true;
    }
    return false;
}
