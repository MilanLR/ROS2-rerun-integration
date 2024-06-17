#!/usr/bin/env python3

import argparse
import sys
import rerun as rr
import cv2
try:
    import cv_bridge
    from numpy.lib.recfunctions import structured_to_unstructured
    import rclpy
    from image_geometry import PinholeCameraModel
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, JointState
    from sensor_msgs_py import point_cloud2
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.msg import ParameterEvent, Log
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from theora_image_transport.msg import Packet
    from realsense2_camera_msgs.msg import Metadata, Extrinsics

except ImportError:
    print(
        """
Could not import the required ROS2 packages.

Make sure you have installed ROS2 (https://docs.ros.org/en/humble/index.html)
and sourced /opt/ros/humble/setup.bash

See: README.md for more details.
"""
    )
    sys.exit(1)

typetotype = {
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CompressedImage": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "std_msgs/msg/String": String,
    "rcl_interfaces/msg/ParameterEvent": ParameterEvent,
    "rosgraph_msgs/msg/Clock": Clock,
    "sensor_msgs/msg/JointState": JointState,
    "rcl_interfaces/msg/Log": Log,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "theora_image_transport/msg/Packet": Packet,
    "realsense2_camera_msgs/msg/Metadata": Metadata,
    "realsense2_camera_msgs/msg/Extrinsics": Extrinsics,
}

class CameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("depth_cam")
        print(self.get_topic_names_and_types())
        print(self.get_node_names())

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_to_frame = {
            "camera": "camera",
            "depth_registered": "depth_registered",
        }

        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        point_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        for (topic, typez) in self.get_topic_names_and_types():
            print(topic)
            typez = typetotype[typez[0]]
            print(f"subbing topic: {topic}, type: {typez}")
            self.create_subscription(
                typez,
                topic,
                lambda x, topic=topic, typez=typez: print(f"topic: {topic}"),
                10 if topic not in ['/camera/depth/points', '/camera/depth_registered/points', '/points'] else point_profile,
                callback_group=self.callback_group,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Camera ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "rerun_camera_ros_node")

    rclpy.init(args=unknownargs)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber, executor=rclpy.executors.MultiThreadedExecutor())

    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

