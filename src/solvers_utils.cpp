#include "moveit_planning/solvers_utils.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_state/robot_state.h>

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group,
                    const geometry_msgs::Pose& target_pose,
                    SolverType solver) {
    switch (solver) {
        case SolverType::OMPL: {
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                return move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            }
            return false;
        }

        case SolverType::CARTESIAN: {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose);
            moveit_msgs::RobotTrajectory trajectory;
            double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            if (fraction > 0.95) {
                return move_group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            }
            return false;
        }

        case SolverType::JOINT_INTERPOLATION: {
            robot_state::RobotState start_state = *move_group.getCurrentState();
            const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup(move_group.getName());
            if (!start_state.setFromIK(joint_model_group, target_pose, 5.0)) return false;

            std::vector<double> joint_values;
            start_state.copyJointGroupPositions(joint_model_group, joint_values);
            move_group.setJointValueTarget(joint_values);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                return move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            }
            return false;
        }
    }
    return false;
}
