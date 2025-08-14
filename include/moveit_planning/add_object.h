#ifndef ADD_OBJECT_H
#define ADD_OBJECT_H

#include <geometry_msgs/Pose.h>
#include <string>

void addObjectToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    const std::string& cube_id,
                    const geometry_msgs::Pose& cube_pose,
                    const std::vector<double>& cube_size,
                    const std::string& frame_id = "panda_link0");

#endif
