#!/usr/bin/env python3

import argparse
import rerun as rr
from rospy import Duration
import cv2
import cv_bridge
from numpy.lib.recfunctions import structured_to_unstructured
import rclpy
from image_geometry import PinholeCameraModel
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
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
            "/camera/camera/depth/image_rect_raw",
            lambda x: self.depth_image_callback(x, "camera/depth"),
            10,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            lambda x: self.image_callback(x, "camera/rgb"),
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
            "/camera/camera/depth/color/points",
            self.point_cloud_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth=1
            ),
            callback_group=self.callback_group,
        )
 
    def depth_image_callback(self, img: Image, rerun_endpoint) -> None:
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Convert the ROS image message to a CV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")
        
        rr.log(rerun_endpoint, rr.DepthImage(cv_image))

    def image_callback(self, img: Image, rerun_endpoint) -> None:
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Convert the ROS image message to a CV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")

        # Check if the image encoding is YUYV
        if img.encoding == "yuyv":
            # Convert YUYV to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUYV)
        elif img.encoding in ["yuv422_yuy2", "yuy2", "yuv422"]:
            # Convert YUV422 (YUY2) to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
        elif img.encoding == "mono8":
            # Handle single channel (grayscale) images
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        elif img.encoding in ["rgb8", "bgr8"]:
            # Handle already RGB/BGR images
            pass
        else:
            raise ValueError(f"Unexpected image encoding: {img.encoding}")

        rr.log(rerun_endpoint, rr.Image(cv_image))

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

        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)

        pts = structured_to_unstructured(pts)

        if "r" in [x.name for x in points.fields]:
            colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)
            colors = structured_to_unstructured(colors)
            rr.log("depth_registered/point_cloud", rr.Points3D(pts, colors=colors))

        else:
            # Log points once rigidly under robot/camera/points. This is a robot-centric
            # view of the world.
            rr.log("depth_registered/point_cloud", rr.Points3D(pts))

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

