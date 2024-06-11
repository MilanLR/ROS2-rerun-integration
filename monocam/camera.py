#!/usr/bin/env python3

import argparse
import sys
import rerun as rr
import cv2
try:
    import cv_bridge
    import rclpy
    from image_geometry import PinholeCameraModel
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo, CompressedImage
    from std_msgs.msg import String
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
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


class CameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("rr_usb_cam")
        print(self.get_topic_names_and_types())
        print(self.get_node_names())

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_to_frame = {
            "camera": "camera",
        }

        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        self.img_sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera_info",
            self.camera_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.compressed_img_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.camera_compressed_callback,
            10,
            callback_group=self.callback_group,
        )

        self.compressed_depth_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.camera_compressed_depth_callback,
            10,
            callback_group=self.callback_group,
        )
 
    def image_callback(self, img: Image) -> None:
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

        rr.log("camera/img", rr.Image(cv_image))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        rr.log("camera/description", rr.TextLog(f"Received camera info, width={msg.width}, height={msg.height}", level=rr.TextLogLevel.INFO))

    def camera_compressed_callback(self, msg: CompressedImage) -> None:
        print(msg.format)
        print(' '.join(format(byte, '08b') for byte in msg.data[:10]))
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        print(cv_image.shape)
        cv_image = cv2.cvtColor(cv_image, cv2.CV_8UC1)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)

        rr.log("camera/compressed", rr.Image(cv_image))

    def camera_compressed_depth_callback(self, msg: CompressedImage) -> None:
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "passthrough")

        rr.log("camera/depth", rr.DepthImage(cv_image))

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

