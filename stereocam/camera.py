#!/usr/bin/env python3

import argparse
import sys
import rerun as rr
import rerun_urdf
import cv2
import cv_bridge
from numpy.lib.recfunctions import structured_to_unstructured
import rclpy
from image_geometry import PinholeCameraModel
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, CompressedImage, PointCloud2, PointField, JointState
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import ParameterEvent, Log
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from theora_image_transport.msg import Packet
from tf2_ros import TransformException


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
}

class CameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("depth_cam")

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



        self.img_sub = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            lambda x: self.raw_image_callback(x, "camera/depth"),
            10,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/camera/ir/image",
            lambda x: self.raw_image_callback(x, "camera/ir"),
            10,
            callback_group=self.callback_group,
        )

        # self.img_sub = self.create_subscription(
        #     Image,
        #     "/camera/rgb/image_raw",
        #     lambda x: self.raw_image_callback(x, "camera/img"),
        #     10,
        #     callback_group=self.callback_group,
        # )

        # self.depth_registered_raw_sub = self.create_subscription(
        #     Image,
        #     "/camera/depth_registered/image_raw",
        #     lambda x: self.raw_image_callback(x, "depth_registered/img"),
        #     10,
        #     callback_group=self.callback_group,
        # )

        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            "/camera/depth/points",
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
        print("Header:", points.header)
        print("Height:", points.height)
        print("Width:", points.width)
        print("Fields:", points.fields)
        print("Is Big Endian:", points.is_bigendian)
        print("Point Step:", points.point_step)
        print("Row Step:", points.row_step)
        print("Is Dense:", points.is_dense)
        print("Data Length:", len(points.data))
        print("Data:", points.data[:20])

        # pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)
        pts = point_cloud2.read_points(points, skip_nans=True)
        print(pts.shape)
        print(pts[0])

        pts = structured_to_unstructured(pts)
        print(pts.shape)
        print(pts[0])

        # Log points once rigidly under robot/camera/points. This is a robot-centric
        # view of the world.
        rr.log("depth_registered/point_cloud", rr.Points3D(pts))
        # self.log_tf_as_transform3d("points", time)

    def colored_point_cloud_callback(self, points: PointCloud2) -> None:

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
        colors = structured_to_unstructured(colors)

        # Log points once rigidly under robot/camera/points. This is a robot-centric
        # view of the world.
        rr.log("depth_registered/point_cloud", rr.Points3D(pts, colors=colors))


    def camera_info_callback(self, msg: CameraInfo) -> None:
        rr.log("camera/description", rr.TextLog(f"Received camera info, width={msg.width}, height={msg.height}", level=rr.TextLogLevel.INFO))


    def log_tf_as_transform3d(self, path: str, time: Time) -> None:
        """Helper to look up a transform with tf and log using `log_transform3d`."""
        # Get the parent path
        parent_path = path.rsplit("/", 1)[0]

        # Find the corresponding frames from the mapping
        child_frame = self.path_to_frame[path]
        parent_frame = self.path_to_frame[parent_path]

        # Do the TF lookup to get transform from child (source) -> parent (target)
        try:
            tf = self.tf_buffer.lookup_transform(parent_frame, child_frame, time, timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(path, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

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

