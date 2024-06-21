from typing import Set
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.time import Time

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured

import rerun as rr

from rerun_visualization.urdf import make_urdf_logger

from tf2_ros import TransformException, LookupException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import laser_geometry
from sensor_msgs_py import point_cloud2


class RerunNode(Node):
    def __init__(self):
        super().__init__("rerun")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.qos_profile = qos_profile

        self.joint_path_map = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.laser_proj = laser_geometry.laser_geometry.LaserProjection()

    def auto_subscribe(self, topics: Set[str] = set()):
        for topic_name, topic_type in self.get_topic_names_and_types():
            topic_type = topic_type[0]
            print(f"topic_name: {topic_name}\ttopic_type: {topic_type}")
            if topic_name not in topics:
                continue

            match topic_type:
                case "sensor_msgs/msg/LaserScan":
                    from sensor_msgs.msg import LaserScan

                    topic_type = LaserScan
                    topic_callback = lambda x,: self.laserscan_callback(x, topic_name)
                case "trajectory_msgs/msg/JointTrajectory":
                    from trajectory_msgs.msg import JointTrajectory

                    topic_type = JointTrajectory
                    topic_callback = lambda x: self.joint_trajectory_callback(
                        x, topic_name
                    )
                case "nav_msgs/msg/Odometry":
                    from nav_msgs.msg import Odometry

                    topic_type = Odometry
                    topic_callback = lambda x: self.odometry_callback(x, topic_name)
                case "sensor_msgs/msg/PointCloud2":
                    from sensor_msgs.msg import PointCloud2

                    topic_type = PointCloud2
                    topic_callback = lambda x: self.point_cloud_callback(x, topic_name)
                case "sensor_msgs/msg/Image":
                    from sensor_msgs.msg import Image

                    topic_type = Image
                    topic_callback = lambda x: self.image_callback(x, topic_name)
                case "sensor_msgs/msg/CompressedImage":
                    from sensor_msgs.msg import Image

                    topic_type = Image
                    topic_callback = lambda x: self.image_callback(x, topic_name)
                case "nav_msgs/msg/OccupancyGrid":
                    from nav_msgs.msg import OccupancyGrid
                    
                    topic_type = OccupancyGrid
                    topic_callback = lambda x: self.map_callback(x, topic_name)
                case _:
                    print(f"topic {topic_type} not configured")
                    continue

            print(f"subscribing to topic: {topic_name}, type: {topic_type}")
            self.create_subscription(
                topic_type,
                topic_name,
                topic_callback,
                self.qos_profile,
            )

    def load_urdf(self, urdf_path: str):
        self.urdf_logger = make_urdf_logger(urdf_path)
        self.joint_path_map = self.urdf_logger.get_joint_path_map()
        self.urdf_logger.log()

    def joint_trajectory_callback(self, msg, topic_name):
        # time = Time.from_msg(msg.header.stamp)
        # rr.set_time_nanos("ros_time", time.nanoseconds)

        def log_joint(joint_name, joint_angle):
            transform = rr.Transform3D(
                rotation=rr.datatypes.RotationAxisAngle(
                    angle=rr.datatypes.Angle(joint_angle)
                ),
            )
            rr.log(joint_name, transform)

        def log_urdf_joint(rerun_path, joint, origin, rotation):
            transform = rr.Transform3D(
                # translation=origin - joint.origin.xyz,
                # translation=origin + joint.origin.xyz,
                translation=origin,
                # translation=np.array(joint.origin.xyz) - origin,
                # rotation=rr.datatypes.RotationAxisAngle(
                #    axis=joint.axis, angle=rr.datatypes.Angle(joint_angle)
                # ),
                rotation=rotation,
            )
            rr.log(rerun_path, transform)

        # time = Time.from_msg(msg.header.stamp)
        # rr.set_time_nanos("ros_time", time.nanoseconds)

        for joint_name, joint_angle in list(
            zip(msg.joint_names, msg.points[0].positions)
        ):
            urdf_joint_data = self.joint_path_map.get(joint_name)
            if urdf_joint_data is None:
                log_joint(joint_name, joint_angle)
            else:
                try:
                    splitted_joint_name = joint_name.split("_")
                    target_frame = splitted_joint_name[0]
                    if target_frame == "base":
                        target_frame = "base_link"
                    source_frame = splitted_joint_name[1]
                    transform_stamped = self.tf_buffer.lookup_transform(
                        # target_frame='base_link',
                        # source_frame=dest,
                        target_frame=target_frame,
                        source_frame=source_frame,
                        time=rclpy.time.Time(),
                    )
                    transform = transform_stamped.transform
                    translation = transform.translation
                    origin = np.array([translation.x, translation.y, translation.z])
                    rotation = transform.rotation
                    rotation = np.array(
                        [rotation.x, rotation.y, rotation.z, rotation.w]
                    )

                except (LookupException, ConnectivityException) as ex:
                    self.get_logger().info(
                        f"Could not transform {source_frame} to {target_frame}: {ex}"
                    )
                    origin = np.zeros(3)
                    rotation = np.zeros((4, 4))

                log_urdf_joint(*urdf_joint_data, origin, rotation)

    def laserscan_callback(self, msg, topic_name):
        # time = Time.from_msg(msg.header.stamp)
        # rr.set_time_nanos("ros_time", time.nanoseconds)

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
        rr.log(topic_name, lines)

        # time = Time.from_msg(msg.header.stamp)
        # rr.set_time_nanos("ros_time", time.nanoseconds)

        # # Project the laser msg to a collection of points
        # points = self.laser_proj.projectLaser(msg)
        # pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)
        # pts = structured_to_unstructured(pts)

        # # Turn every pt into a line-segment from the origin to the point.
        # origin = (pts / np.linalg.norm(pts, axis=1).reshape(-1, 1)) * 0.3
        # segs = np.hstack([origin, pts]).reshape(pts.shape[0] * 2, 3)

        # rr.log(topic_name, rr.LineStrips3D(segs, radii=0.0025))

    def odometry_callback(self, msg, topic_name):
        # time = Time.from_msg(msg.header.stamp)
        # rr.set_time_nanos("ros_time", time.nanoseconds)

        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        translation = np.array([position.x, position.y, position.z])
        rotation = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self._odometry_translation = translation
        self._odometry_rotation = rotation

        rr.log(topic_name, rr.Points3D([translation]))

    def depth_image_callback(self, img, topic_name) -> None:
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Convert the ROS image message to a CV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")

        rr.log(topic_name, rr.DepthImage(cv_image))

    def image_callback(self, img, topic_name) -> None:
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

        rr.log(topic_name, rr.Image(cv_image))

    def point_cloud_callback(self, points, topic_name) -> None:
        # print("pointing at cloudss")
        # print("Header:", points.header)
        # print("Height:", points.height)
        # print("Width:", points.width)
        print("Fields:", points.fields)
        # print("Is Big Endian:", points.is_bigendian)
        # print("Point Step:", points.point_step)
        # print("Row Step:", points.row_step)
        # print("Is Dense:", points.is_dense)
        # print("Data Length:", len(points.data))
        # print("Data:", points.data[:20])

        pts = point_cloud2.read_points(
            points, field_names=["x", "y", "z"], skip_nans=True
        )

        pts = structured_to_unstructured(pts)

        if "rgb" in [x.name for x in points.fields]:
            points.fields = [
                PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
                PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
                PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
            ]
            colors = point_cloud2.read_points(
                points, field_names=["r", "g", "b"], skip_nans=True
            )
            colors = structured_to_unstructured(colors)
            rr.log(topic_name, rr.Points3D(pts, colors=colors))

        else:
            # Log points once rigidly under robot/camera/points. This is a robot-centric
            # view of the world.
            rr.log(topic_name, rr.Points3D(pts))

    def map_callback(self, msg, topic_name):
        image_data = np.ones((msg.info.height, msg.info.width, 3), dtype=np.uint8)

        # transform = self.tf_buffer.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=0.5))

        for row in range(msg.info.height):
            for col in range(msg.info.width):
                value = msg.data[row * msg.info.width + col]
                if value == -1:
                    image_data[row, col] = [255, 0, 0]  # Red for -1
                else:
                    # Interpolate between black and white based on value
                    intensity = int((value / 100) * 255)
                    image_data[row, col] = [intensity, intensity, intensity]

        # Log the occupancy grid as an image
        rr.log(
            f"{topic_name}/occupancy",
            rr.Image(image_data),
        )

    def urdf_callback(self, urdf_msg, topic_name: str) -> None:
        """Log a URDF using `log_scene` from `rerun_urdf`."""
        urdf = rerun_urdf.load_urdf_from_msg(urdf_msg)

        # The turtlebot URDF appears to have scale set incorrectly for the camera-link
        # Although rviz loads it properly `yourdfpy` does not.
        # orig, _ = urdf.scene.graph.get("camera_link")
        # scale = trimesh.transformations.scale_matrix(0.00254)
        # urdf.scene.graph.update(frame_to="camera_link", matrix=orig.dot(scale))
        scaled = urdf.scene.scaled(1.0)

        rerun_urdf.log_scene(scene=scaled, node=urdf.base_link, path=topic_name, static=True)

def main(args=None):
    rr.init("pupper")
    rr.connect("10.0.8.92:9876")

    rclpy.init(args=args)

    topics_to_subscribe_to = {
        "/odom",
        "/tf",
        "/scan",
        "/joint_group_effort_controller/joint_trajectory",
        "/map"
    }
    rerun_node = RerunNode()
    rerun_node.get_logger().info("Hello friend!")
    # rerun_node.load_urdf(
    #     "/home/ubuntu/mini_pupper_ros_urdf/mini_pupper_description/urdf/mini_pupper_description.urdf"
    # )
    rerun_node.auto_subscribe(topics=topics_to_subscribe_to)

    rclpy.spin(rerun_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rerun_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
