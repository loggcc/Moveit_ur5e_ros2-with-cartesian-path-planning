#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto move_group_node = std::make_shared<rclcpp::Node>(
    "move_group_interface_tutorial",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {
    executor.spin();
  }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur5e_arm";
  // static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  RCLCPP_INFO(LOGGER, "Initializing MoveGroupInterface...");
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP_ARM);
  move_group.setEndEffectorLink("tool0");
  // moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, PLANNING_GROUP_GRIPPER);

  auto current_state_arm = move_group.getCurrentState(10);
  if (!current_state_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to get current robot state.");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(LOGGER, "debug2");
  const moveit::core::JointModelGroup *joint_model_group_arm = current_state_arm->getJointModelGroup(PLANNING_GROUP_ARM);
  if (!joint_model_group_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to get JointModelGroup for '%s'.", PLANNING_GROUP_ARM.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  RCLCPP_INFO(LOGGER, "Planning to home position...");
  joint_group_positions_arm[1]= -2.5;
  joint_group_positions_arm[2]= 1.50;
  joint_group_positions_arm[3]= -1.50;
  joint_group_positions_arm[4]= -1.55;

  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Home plan %s", success_arm ? "succeeded" : "failed");
  if (success_arm) move_group.execute(my_plan_arm);

  RCLCPP_INFO(LOGGER, "Planning to pregrasp position...");
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.34;
  target_pose1.position.y = 0.1;
  target_pose1.position.z = 0.264;
  move_group.setPoseTarget(target_pose1);

  success_arm = (move_group.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Pregrasp plan %s", success_arm ? "succeeded" : "failed");
  if (success_arm) move_group.execute(my_plan_arm);

  RCLCPP_INFO(LOGGER, "Approaching object...");
  target_pose1.position.z -= 0.04;
  move_group.setPoseTarget(target_pose1);

  success_arm = (move_group.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Approach plan %s", success_arm ? "succeeded" : "failed");
  if (success_arm) move_group.execute(my_plan_arm);

  RCLCPP_INFO(LOGGER, "Retreating from object...");
  target_pose1.position.z += 0.04;
  move_group.setPoseTarget(target_pose1);

  success_arm = (move_group.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Retreat plan %s", success_arm ? "succeeded" : "failed");
  if (success_arm) move_group.execute(my_plan_arm);

  rclcpp::shutdown();
  return 0;
}
