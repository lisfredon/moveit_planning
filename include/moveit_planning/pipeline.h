#pragma once
#include "config_utils.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class GraspPipeline {
public:
    GraspPipeline(const GraspConfig& cfg);
    bool run();

private:
    bool approachPhase();
    bool gripPhase();
    void closeGripper();
    void attachObject();

    const GraspConfig& cfg_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    moveit::planning_interface::PlanningSceneInterface scene_;
};
