#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_cpp_ur5e_node");

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur5e_arm");

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x += 0.0;
  target_pose.position.y += 0.3;
  target_pose.position.z += -0.1;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  target_pose.orientation = tf2::toMsg(q);
  move_group.setPoseTarget(target_pose);

  // Marker for constraint box
  visualization_msgs::msg::Marker box;
  box.header.frame_id = "base_link";
  box.header.stamp = node->now();
  box.ns = "constraints";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::CUBE;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.pose.position.x = current_pose.position.x;
  box.pose.position.y = 0.35;
  box.pose.position.z = 0.35;
  box.pose.orientation.w = 1.0;
  box.scale.x = 0.6;
  box.scale.y = 1.0;
  box.scale.z = 1.0;
  box.color.r = 0.0;
  box.color.g = 0.0;
  box.color.b = 1.0;
  box.color.a = 0.4;

  // Marker for target pose
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "base_link";
  arrow.header.stamp = node->now();
  arrow.ns = "target";
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.pose = target_pose;
  arrow.scale.x = 0.1;
  arrow.scale.y = 0.02;
  arrow.scale.z = 0.02;
  arrow.color.r = 0.0;
  arrow.color.g = 1.0;
  arrow.color.b = 0.0;
  arrow.color.a = 1.0;

  marker_pub->publish(box);
  marker_pub->publish(arrow);

  move_group.setPlanningTime(10.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group.execute(plan);
  }

  rclcpp::shutdown();
  return 0;
}

