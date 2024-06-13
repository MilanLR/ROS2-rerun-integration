import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import math

from pathlib import Path

import rerun as rr

from rerun_visualization.urdf import make_urdf_logger

rr.init("pupper")
rr.connect("10.0.8.92:9876")

typetotype = {
    "sensor_msgs/msg/LaserScan": LaserScan,
    "trajectory_msgs/msg/JointTrajectory": JointTrajectory,
    #"sensor_msgs/msg/Image": Image,
    #"sensor_msgs/msg/CompressedImage": Image,
    #"sensor_msgs/msg/CameraInfo": CameraInfo,
    #"sensor_msgs/msg/PointCloud2": PointCloud2,
    #"std_msgs/msg/String": String,
    #"rcl_interfaces/msg/ParameterEvent": ParameterEvent,
    #"rosgraph_msgs/msg/Clock": Clock,
    #"sensor_msgs/msg/JointState": JointState,
    #"rcl_interfaces/msg/Log": Log,
    #"tf2_msgs/msg/TFMessage": TFMessage,
    #"theora_image_transport/msg/Packet": Packet,
}

class RerunNode(Node):
    def __init__(self):
        super().__init__("rerun")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.qos_profile = qos_profile

        self.joint_path_map = {}

        self.supported_topics = {
            LaserScan: self.laserscan_callback,
            JointTrajectory: self.joint_trajectory_callback,
        }

        #self.subscription = self.create_subscription(
        #    LaserScan, "scan", self.listener_callback, qos_profile
        #)

        #self.subscription = self.create_subscription(
        #    JointTrajectory,
        #    "joint_group_effort_controller/joint_trajectory",
        #    self.listener_callback_joints,
        #    qos_profile,
        #)

    
    def auto_subscribe(self):
        for (topic_name, topic_type) in self.get_topic_names_and_types():
            print(topic_name)
            topic_type = typetotype.get(topic_type[0])

            topic_callback = self.get_topic_callback(topic_type)
            if topic_callback == None:
                continue

            print(f"subbing topic: {topic_name}, type: {topic_type}")
            self.create_subscription(
                topic_type,
                topic_name,
                topic_callback,
                self.qos_profile,
            )


    def get_topic_callback(self, topic_type):
        return self.supported_topics.get(topic_type)


    def load_urdf(self, urdf_path: str):
        self.urdf_logger = make_urdf_logger(urdf_path)
        self.joint_path_map = self.urdf_logger.get_joint_path_map()
        self.urdf_logger.log()


    def joint_trajectory_callback(self, msg: JointTrajectory):
        def log_joint(joint_name, joint_angle):
            transform = rr.Transform3D(
                rotation=rr.datatypes.RotationAxisAngle(
                    angle=rr.datatypes.Angle(joint_angle)
                ),
            )
            rr.log(joint_name, transform)

        def log_urdf_joint(rerun_path, joint):
            transform = rr.Transform3D(
                translation=joint.origin.xyz,
                rotation=rr.datatypes.RotationAxisAngle(
                    axis=joint.axis, angle=rr.datatypes.Angle(joint_angle)
                ),
            )
            rr.log(rerun_path, transform)

        for joint_name, joint_angle in list(
            zip(msg.joint_names, msg.points[0].positions)
        ):
            urdf_joint_data = self.joint_path_map.get(joint_name)
            if urdf_joint_data is None:
                log_joint(joint_name, joint_angle)
            else:
                log_urdf_joint(*urdf_joint_data)


    def laserscan_callback(self, msg: LaserScan):
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

    rerun_node = RerunNode()
    rerun_node.get_logger().info("Hello friend!")
    rerun_node.load_urdf("/home/ubuntu/mini_pupper_ros_urdf/mini_pupper_description/urdf/mini_pupper_description.urdf")
    rerun_node.auto_subscribe()

    rclpy.spin(rerun_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rerun_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
