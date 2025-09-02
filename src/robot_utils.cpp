#include "moveit_planning/robot_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

PickPlaceManager initRobot(const ros::NodeHandle& nh)
{
    std::string robot_name;
    nh.param<std::string>("/robot", robot_name, "");

    std::string manipulator_group, gripper_group;
    nh.param<std::string>("/robots/" + robot_name + "/manipulator_group", manipulator_group, "");
    nh.param<std::string>("/robots/" + robot_name + "/gripper_group", gripper_group, "");

    if (manipulator_group.empty() || gripper_group.empty()) {
        ROS_FATAL("Impossible de charger les groupes MoveIt pour le robot '%s'. Vérifie robots.yaml !", robot_name.c_str());
        throw std::runtime_error("Groupes MoveIt non définis");
    }

    PickPlaceManager manager(manipulator_group, gripper_group);
    manager.openGripper();

    ROS_INFO_STREAM("[InitRobot] Robot '" << robot_name << "' initialisé avec : "
                    << "manipulator='" << manipulator_group
                    << "', gripper='" << gripper_group << "'");

    return manager;
}

bool attachObject(moveit::planning_interface::MoveGroupInterface& move_group,
                  const std::string& object_id,
                  const std::string& hand_link,
                  const std::vector<std::string>& touch_links) 
{
    move_group.attachObject(object_id, hand_link, touch_links);
    ros::Duration(0.5).sleep(); // laisser la scene se mettre à jour
    ROS_INFO("Objet %s attaché à %s", object_id.c_str(), hand_link.c_str());
    return true;
}

std::vector<std::string> getGroupLinks(const std::string& group_name) {
    static robot_model_loader::RobotModelLoader loader("robot_description");
    static moveit::core::RobotModelPtr kinematic_model = loader.getModel();
    if (!kinematic_model) {
        ROS_ERROR("Impossible de charger le modèle du robot");
        return {};
    }

    const auto* jmg = kinematic_model->getJointModelGroup(group_name);
    if (!jmg) {
        ROS_ERROR("Groupe %s introuvable", group_name.c_str());
        return {};
    }

    return jmg->getLinkModelNames();
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
    ROS_INFO("Objet %s détaché", object_id.c_str());
}
