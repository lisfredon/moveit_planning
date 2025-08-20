#ifndef MOVEIT_TASK_CONSTRUCTOR_DEMO_PICK_PLACE_TASK_H
#define MOVEIT_TASK_CONSTRUCTOR_DEMO_PICK_PLACE_TASK_H

#include <moveit/task_constructor/task.h>
#include <ros/ros.h>

namespace moveit_task_constructor_demo {

class PickPlaceTask {
public:
  PickPlaceTask(const std::string& task_name, const ros::NodeHandle& pnh);

  bool init();
  bool plan();
  bool execute();

private:
  void loadParameters();
  ros::NodeHandle pnh_;
  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;
};

}  // namespace moveit_task_constructor_demo

#endif
