#include <memory>
#include <thread>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur5e_arm");

  // debug: check if the move group is nitilaized correctly  done!
  if (!move_group_interface.getRobotModel()->hasJointModelGroup("ur5e_arm")) {
    RCLCPP_ERROR(logger, "Joint model group 'ur5e_arm' not found in robot model.");
  } else {
    RCLCPP_INFO(logger, "Successfully connected to joint model group 'ur5e_arm'.");
  }
 // // //

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_visual_tools::MoveItVisualTools moveit_visual_tools(
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto draw_title = [&moveit_visual_tools](const std::string& text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto prompt = [&moveit_visual_tools](const std::string& text) {
    moveit_visual_tools.prompt(text);
  };

  auto draw_trajectory_tool_path = [&moveit_visual_tools,
    jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur5e_arm")]
    (const moveit_msgs::msg::RobotTrajectory& trajectory) {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };



  // auto const target_pose = [] {
  //   geometry_msgs::msg::Pose msg;

  //   // 180 deg rotation about X-axis → gripper facing down
  //   tf2::Quaternion q;
  //   q.setRPY(M_PI, 0, 0);  // roll = π, pitch = 0, yaw = 0
  //   msg.orientation.x = q.x();
  //   msg.orientation.y = q.y();
  //   msg.orientation.z = q.z();
  //   msg.orientation.w = q.w();

  //   msg.position.x = -0.5;
  //   msg.position.y = -0.1;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();
  
  geometry_msgs::msg::Pose target_pose;
  double x, y, z;
  std::cout << "Enter target x y z (space-separated): ";
  std::cin >> x >> y >> z;

  // Orientation: gripper facing down (180° about X-axis)
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;


  move_group_interface.setEndEffectorLink("tool0");

  move_group_interface.setPoseTarget(target_pose);


  moveit_visual_tools.publishCuboid(
      target_pose,
      0.03, 0.03, 0.03,
      rviz_visual_tools::GREEN
  );
  moveit_visual_tools.trigger();  // Ensure the marker is actually sent


    
  // Add collision object
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.6;
    box_pose.position.y = -0.5;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  
  planning_scene_interface.applyCollisionObjects({collision_object});

  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(logger, "Attempting to plan to target pose...");
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Planning succeeded. Trajectory has %lu points.",
                plan.trajectory_.joint_trajectory.points.size());
  } else {
    RCLCPP_ERROR(logger, "Planning failed. No valid trajectory found.");
  }


  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
