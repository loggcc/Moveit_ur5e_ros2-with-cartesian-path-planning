#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>


#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <iostream>  // For std::cin and std::cout
const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();


  static const std::string PLANNING_GROUP_ARM = "ur5e_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP_ARM);
  move_group.setEndEffectorLink("tool0");


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::string frame_id = move_group.getPlanningFrame();

// setting constraints
  // // Box 1
  // moveit_msgs::msg::CollisionObject box1;
  // box1.header.frame_id = frame_id;
  // box1.id = "box1";


  // shape_msgs::msg::SolidPrimitive primitive1;
  // primitive1.type = primitive1.BOX;
  // primitive1.dimensions = {1.0, 0.1, 1.0};


  // geometry_msgs::msg::Pose pose1;
  // pose1.orientation.w = 1.0;
  // pose1.position.x = -0.6;
  // pose1.position.y = -0.4;
  // pose1.position.z = 0.5;


  // box1.primitives.push_back(primitive1);
  // box1.primitive_poses.push_back(pose1);
  // box1.operation = box1.ADD;
  // collision_objects.push_back(box1);





// Apply constraints
  // planning_scene_interface.applyCollisionObjects(collision_objects);




 RCLCPP_INFO(LOGGER, "Approaching target pose with Cartesian path...");


// Define target pose
 geometry_msgs::msg::Pose target_pose1;
 double x, y, z;
 std::cout << "Enter target x y z (space-separated): ";
 std::cin >> x >> y >> z;


target_pose1.orientation.x = -1.0;
target_pose1.orientation.y = 0.0;
target_pose1.orientation.z = 0.0;
target_pose1.orientation.w = 0.0;
target_pose1.position.x = x;
target_pose1.position.y = y;
target_pose1.position.z = z;






// Generate waypoints for Cartesian motion
std::vector<geometry_msgs::msg::Pose> waypoints;
geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
waypoints.push_back(start_pose);


// Intermediate step if needed
geometry_msgs::msg::Pose mid_pose = target_pose1;
mid_pose.position.z += 0.05;  // offset above final z to ensure vertical descent
waypoints.push_back(mid_pose);


// Final target
waypoints.push_back(target_pose1);


// Plan Cartesian path
moveit_msgs::msg::RobotTrajectory trajectory;
const double eef_step = 0.01;     // resolution (meters)
const double jump_threshold = 0.0;  // disable jump detection
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);


if (fraction > 0.99) {


  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;


  // original trajectory points
  for (const auto& point : trajectory.joint_trajectory.points) {
        for (const auto& posit : point.positions) {
            std::cout << posit << ", ";
        }
        std::cout << std::endl;
    }




  std::cout << "**************************************** second vel  ****************" << std::endl;


  // time parameterized the trajectory
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(rt, 0.03, 0.03); // 0.01 is the velocity and acceleration factor

  // turn the time parameterized trajectory into message 
  rt.getRobotTrajectoryMsg(trajectory);


  // output the time parameterized trajectory points
  for (const auto& point : trajectory.joint_trajectory.points) {
      for (const auto& posit : point.positions) {
          std::cout << posit << ", ";
      }
      std::cout << std::endl;
  }


  plan.trajectory_ = trajectory;
  // motion execution
  
  
  move_group.execute(plan);
} else {
  RCLCPP_WARN(LOGGER, "Cartesian path planning failed, only achieved %.2f%% of the path", fraction * 100.0);
}







  rclcpp::shutdown();
  return 0;
}






