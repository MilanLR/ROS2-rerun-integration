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
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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
}

class CameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("depth_cam")
        print(self.get_topic_names_and_types())
        print(self.get_node_names())

        for (topic, typez) in self.get_topic_names_and_types():
            typez =  typetotype[typez]
            self.img_sub = self.create_subscription(
                typez,
                topic,
                lambda x: print(f"topic: {topic}, type: {typez}, data: {x}"),
                10,
                callback_group=self.callback_group,
            )


        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_to_frame = {
            "camera": "camera",
            "depth_registered": "depth_registered",
        }

        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        self.img_sub = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            lambda x: self.raw_image_callback(x, "camera/depth"),
            10,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/camera/rgb/image_raw",
            lambda x: self.raw_image_callback(x, "camera/img"),
            10,
            callback_group=self.callback_group,
        )

        # self.depth_registered_raw_sub = self.create_subscription(
        #     Image,
        #     "/camera/depth_registered/image_raw",
        #     lambda x: self.raw_image_callback(x, "depth_registered/img"),
        #     10,
        #     callback_group=self.callback_group,
        # )

        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            "/camera/depth_registered/points",
            self.point_cloud_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth=1
            ),
            callback_group=self.callback_group,
        )
 
    def raw_image_callback(self, img: Image, rerun_endpoint) -> None:
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Convert the ROS image message to a CV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")
        
        rr.log(rerun_endpoint, rr.DepthImage(cv_image))
    
    def point_cloud_callback(self, points: PointCloud2) -> None:
        print("pointing at cloudss")
        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)

        # The realsense driver exposes a float field called 'rgb', but the data is actually stored
        # as bytes within the payload (not a float at all). Patch points.field to use the correct
        # r,g,b, offsets so we can extract them with read_points.
        points.fields = [
            PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
        ]

        colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)

        pts = structured_to_unstructured(pts)
        colors = colors = structured_to_unstructured(colors)

        # Log points once rigidly under robot/camera/points. This is a robot-centric
        # view of the world.
        rr.log("depth_registered/point_cloud", rr.Points3D(pts, colors=colors))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        rr.log("camera/description", rr.TextLog(f"Received camera info, width={msg.width}, height={msg.height}", level=rr.TextLogLevel.INFO))


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

