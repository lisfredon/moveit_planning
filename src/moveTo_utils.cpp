#include "moveit_planning/moveTo_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

bool moveTo(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose_in,
    SolverType solver,
    const std::string& phase_name)
{
    // Normaliser orientation
    tf2::Quaternion q_goal;
    tf2::fromMsg(target_pose_in.orientation, q_goal);
    q_goal.normalize();

    tf2::Quaternion q_current;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q_current);

    if (q_goal.dot(q_current) < 0.0) {
        q_goal = tf2::Quaternion(-q_goal.x(), -q_goal.y(), -q_goal.z(), -q_goal.w());
    }

    geometry_msgs::Pose target_pose = target_pose_in;
    target_pose.orientation = tf2::toMsg(q_goal);

    // ✅ Convertir la pose en solution IK
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(move_group.getRobotModel()));
    const robot_state::JointModelGroup* joint_model_group =
        kinematic_state->getJointModelGroup(move_group.getName());

    bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose);


    if (!found_ik) {
        ROS_ERROR_STREAM("IK non trouvé pour la phase " << phase_name);
        return false;
    }

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // ✅ Utiliser joint goals au lieu de pose goals
    move_group.setJointValueTarget(joint_values);

    // Exécution
    if (!planAndExecute(move_group, target_pose, solver)) {
        ROS_ERROR_STREAM("Impossible de planifier la phase de " << phase_name << " !");
        return false;
    }

    ROS_INFO_STREAM("Phase de " << phase_name << " effectuée.");
    return true;
}
