#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class UR5eDirectCommand(Node):
    def __init__(self):
        super().__init__('ur5e_direct_command')

        # Publisher to scaled controller (real robot)
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Store the timer handle explicitly
        self.timer = self.create_timer(1.0, self.send_command)

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        point.time_from_start.sec = 3

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info("Sent direct joint command to UR5e")

        # Correct way to destroy the timer
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)
    node = UR5eDirectCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

