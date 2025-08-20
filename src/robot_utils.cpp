#include "moveit_planning/robot_utils.h"

bool attachObject(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& object_id,
    const std::string& hand_link,
    const std::vector<std::string>& touch_links)
{
    try {
        move_group.attachObject(object_id, hand_link, touch_links);
        ros::Duration(0.5).sleep(); // attendre que le PlanningScene soit à jour
        ROS_INFO("Objet %s attaché à %s", object_id.c_str(), hand_link.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Erreur lors de l'attachement de l'objet %s : %s", object_id.c_str(), e.what());
        return false;
    }
}

void openGripper(moveit::planning_interface::MoveGroupInterface& gripper_group) {
    const std::vector<double> open_position{0.04, 0.04}; 

    gripper_group.setJointValueTarget(open_position);
    gripper_group.move();
}

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double target) {
    std::vector<double> gripper_targets = {target, target};
    gripper_group.setJointValueTarget(gripper_targets);
    gripper_group.move();
}

double getMaxFingerOpening(moveit::planning_interface::MoveGroupInterface& gripper_group) {
    auto joint_model_group = gripper_group.getRobotModel()->getJointModelGroup(gripper_group.getName());
    const std::vector<const moveit::core::JointModel*>& joints = joint_model_group->getActiveJointModels();

    double max_opening = 0.0;
    for (const auto& joint : joints) {
        const auto& bounds = joint->getVariableBounds(joint->getName());
        if (bounds.position_bounded_) {
            double joint_max = bounds.max_position_;
            if (joint_max > max_opening) max_opening = joint_max;
        }
    }
    return max_opening*2;
}

void detachObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  const std::string& object_id) {
    move_group.detachObject(object_id);

}
