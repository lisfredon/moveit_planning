#pragma once
#include "moveit_planning/arm_controller.h"
#include "moveit_planning/gripper_controller.h"
#include "moveit_planning/scene_manager.h"
#include "moveit_planning/solvers_utils.h"

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <string>

class PickPlaceManager {
public:
    PickPlaceManager(const std::string& arm_group_name,
                     const std::string& gripper_group_name);

    

    void openGripper();
    void closeGripper(double target = 0.0);

    void addObject(const std::string& object_id,
                   const geometry_msgs::Pose& pose,
                   const std::vector<double>& size);

    bool pick(const std::string& object_id,
              const geometry_msgs::Pose& obj_pose,
              const std::vector<double>& obj_size,
              const tf2::Vector3& n_local,
              const tf2::Vector3& in_plane_axis,
              int face_index);

    bool place(const std::string& object_id,
                const geometry_msgs::Pose& object_pose,
                geometry_msgs::Pose goal_pose,
                SolverType solver_name);

    moveit::planning_interface::MoveGroupInterface& getArmGroup();
    moveit::planning_interface::MoveGroupInterface& getGripperGroup();


private:
    ArmController arm_;
    GripperController gripper_;
    SceneManager scene_;
};