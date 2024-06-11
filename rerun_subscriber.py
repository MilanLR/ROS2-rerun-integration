import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import matplotlib
import sys
import math

from pathlib import Path

import rerun as rr

from mini_pupper_rerun.urdf import make_urdf_logger

rr.init("pupper")
rr.connect("10.0.8.92:9876")


class RerunSubscriber(Node):
    def __init__(self):
        super().__init__("rerun")

        self.urdf_logger = make_urdf_logger(
            "/home/ubuntu/mini_pupper_ros_urdf/mini_pupper_description/urdf/mini_pupper_description.urdf"
        )
        self.joint_path_map = self.urdf_logger.get_joint_path_map()
        self.urdf_logger.log()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.listener_callback, qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.cmap = matplotlib.colormaps["turbo_r"]

        self.subscription = self.create_subscription(
            JointTrajectory,
            "joint_group_effort_controller/joint_trajectory",
            self.listener_callback_joints,
            qos_profile,
        )

    def listener_callback_joints(self, msg: JointTrajectory):
        self.get_logger().info(f"joint_names:    {msg.joint_names}", once=True)
        self.get_logger().info(f"joint_map:      {self.joint_path_map}", once=True)

        # arrow = rr.Arrows3D(origins=[0, 0, 0], vectors=[0, 1, 0])
        # rr.log("base", arrow)
        # rr.log("base/translated", rr.Transform3D(translation=[1, 0, 0]))
        # rr.log("base/translated", arrow)

        for joint_name, joint_angle in list(
            zip(msg.joint_names, msg.points[0].positions)
        ):
            (rerun_path, joint) = self.joint_path_map[joint_name]
            transform = rr.Transform3D(
                translation=joint.origin.xyz,
                rotation=rr.datatypes.RotationAxisAngle(
                    axis=joint.axis, angle=rr.datatypes.Angle(joint_angle)
                ),
            )
            rr.log(rerun_path, transform)

        # transform = rr.Transform3D(translation=[1, 0, 0])
        # rr.log("/mini_pupper_description.urdf/base_link/lb1/lb2", transform)

    def listener_callback(self, msg: LaserScan):
        self.get_logger().info(f"min_angle:      {msg.angle_min}", once=True)
        self.get_logger().info(f"max_angle:      {msg.angle_max}", once=True)
        self.get_logger().info(f"angle_increment:{msg.angle_increment}", once=True)
        self.get_logger().info(f"time_increment: {msg.time_increment}", once=True)
        self.get_logger().info(f"scan_time:      {msg.scan_time}", once=True)
        self.get_logger().info(f"range_min:      {msg.range_min}", once=True)
        self.get_logger().info(f"range_max:      {msg.range_max}", once=True)
        self.get_logger().info(f"range_len:      {len(msg.ranges)}", once=True)

        ranges = np.array(msg.ranges)
        lin_space = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        x_values_1 = np.cos(lin_space) * msg.range_min
        y_values_1 = np.sin(lin_space) * msg.range_min
        x_values_2 = np.cos(lin_space) * ranges
        y_values_2 = np.sin(lin_space) * ranges

        line_points = np.vstack([y_values_1, x_values_1, y_values_2, x_values_2])
        lines = line_points.T.reshape(len(ranges), 2, 2)
        lines = rr.LineStrips2D(lines)
        rr.log("lidar", lines)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RerunSubscriber()
    minimal_subscriber.get_logger().info("Hello friend!")

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
