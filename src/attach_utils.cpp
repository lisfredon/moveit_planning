#include "moveit_planning/attach_utils.h"

void attachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  const moveit_msgs::CollisionObject& object) {
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = move_group.getEndEffectorLink();
    attached_object.object = object;
    attached_object.object.operation = attached_object.object.ADD;
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
}
