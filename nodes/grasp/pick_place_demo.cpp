#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_go_ready");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_manipulator");
  move_group.setPlanningTime(10.0);

  // Aller à la posture "ready"
  move_group.setNamedTarget("ready");
  ROS_INFO("🟡 Tentative de planification vers la posture 'ready'...");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    ROS_INFO("✅ Planification vers 'ready' réussie. Exécution...");
    move_group.execute(plan);
  } else {
    ROS_ERROR("❌ Échec de la planification vers 'ready'.");
  }

  ros::shutdown();
  return 0;
}
