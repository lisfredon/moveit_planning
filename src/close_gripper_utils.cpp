#include "moveit_planning/close_gripper_utils.h"

double getFingerTarget(const std::vector<double>& cube_size, int face_index) {
    switch(face_index) {
        case 0: case 1: return cube_size[0] / 2.0;
        case 2: case 3: return cube_size[1] / 2.0;
        case 4: case 5: return cube_size[2] / 2.0;
    }
    return 0.0;
}

void closeGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double target) {
    std::vector<double> gripper_targets = {target, target};
    gripper_group.setJointValueTarget(gripper_targets);
    gripper_group.move();
}