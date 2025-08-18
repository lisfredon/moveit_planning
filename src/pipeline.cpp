/*
#include "moveit_planning/pipeline.h"
#include "moveit_planning/solvers_utils.h"  // suppose que tu as déjà ça
#include "moveit_planning/grasp_utils.h"    // suppose que tu as déjà ça

GraspPipeline::GraspPipeline(const GraspConfig& cfg)
: cfg_(cfg),
  arm_group_(cfg.arm_group),
  gripper_group_(cfg.gripper_group) {
    moveit_msgs::CollisionObject cube;
    cube.header.frame_id = "panda_link0";
    cube.id = "cube";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions = cfg.cube_size;
    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(cfg.cube_pose);
    cube.operation = moveit_msgs::CollisionObject::ADD;
    scene_.applyCollisionObjects({cube});
}

bool GraspPipeline::approachPhase() {
    auto pose = generateGraspPose(cfg_.cube_pose, cfg_.face_normals[0], {0,0,1}, cfg_.approach_offset);
    return planAndExecute(arm_group_, pose, SolverType::CARTESIAN);
}

bool GraspPipeline::gripPhase() {
    auto pose = generateGraspPose(cfg_.cube_pose, cfg_.face_normals[0], {0,0,1}, cfg_.grip_offset);
    return planAndExecute(arm_group_, pose, SolverType::OMPL);
}

void GraspPipeline::closeGripper() {
    gripper_group_.setJointValueTarget(std::vector<double>{0.0, 0.0});
    gripper_group_.move();
}

void GraspPipeline::attachObject() {
    moveit_msgs::AttachedCollisionObject attached;
    attached.link_name = arm_group_.getEndEffectorLink();
    attached.object.id = "cube";
    attached.object.operation = moveit_msgs::CollisionObject::ADD;
    scene_.applyAttachedCollisionObject(attached);
}

bool GraspPipeline::run() {
    if (!approachPhase()) return false;
    if (!gripPhase()) return false;
    closeGripper();
    attachObject();
    return true;
}
*/