#include "ur_controllers/scaled_joint_trajectory_controller.hpp"
// 其他头文件...

namespace ur_controllers
{
controller_interface::return_type ScaledJointTrajectoryController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // 其他代码...

  // 行 140：修复 rt_has_pending_goal_ 的逻辑运算
  if (current_external_msg != *new_external_msg && 
      (*rt_has_pending_goal_.readFromRT() && !active_goal) == false) {
    // 内部逻辑...
  }

  // 行 207：修复 rt_is_holding_ 的取反
  if (!before_last_point && !(*rt_is_holding_.readFromRT()) && 
      cmd_timeout_ > 0.0 && time_difference > cmd_timeout_) {
    // 内部逻辑...
  }

  // 行 221：修复 rt_is_holding_ 的取反
  if ((before_last_point || first_sample) && !(*rt_is_holding_.readFromRT()) &&
      // 其他条件...
      ) {
    // 内部逻辑...
  }

  // 行 227：修复 rt_is_holding_ 的取反
  if (!before_last_point && !(*rt_is_holding_.readFromRT()) &&
      // 其他条件...
      ) {
    // 内部逻辑...
  }

  // 行 297：修复 rt_has_pending_goal_ 的赋值
  rt_has_pending_goal_.writeFromNonRT(false);

  // 行 313：修复 rt_has_pending_goal_ 的赋值
  rt_has_pending_goal_.writeFromNonRT(false);

  // 行 330：修复 rt_has_pending_goal_ 的赋值
  rt_has_pending_goal_.writeFromNonRT(false);

  // 行 338：修复 rt_has_pending_goal_ 的取反
  } else if (tolerance_violated_while_moving && !(*rt_has_pending_goal_.readFromRT())) {
    // 内部逻辑...
  }

  // 行 344：修复 rt_has_pending_goal_ 的取反
  } else if (!before_last_point && !within_goal_time && !(*rt_has_pending_goal_.readFromRT())) {
    // 内部逻辑...
  }

  // 其他代码...
  return controller_interface::return_type::OK;
}
}  // namespace ur_controllers

