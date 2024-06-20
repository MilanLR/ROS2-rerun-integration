import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import numpy as np

import rerun as rr

from rerun_visualization.urdf import make_urdf_logger

rr.init("pupper")
rr.connect("10.0.8.92:9876")


class RerunNode(Node):
    def __init__(self):
        super().__init__("rerun")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.qos_profile = qos_profile

        self.joint_path_map = {}
    
    def auto_subscribe(self):
        for (topic_name, topic_type) in self.get_topic_names_and_types():
            match topic_name:
                case "sensor_msgs/msg/LaserScan":
                    from sensor_msgs.msg import LaserScan
                    topic_type = LaserScan
                    topic_callback = self.laserscan_callback
                case "trajectory_msgs/msg/JointTrajectory":
                    from trajectory_msgs.msg import JointTrajectory
                    topic_type = JointTrajectory
                    topic_callback = self.joint_trajectory_callback
                case "nav_msgs/msg/Odometry":
                    from nav_msgs.msg import Odometry
                    topic_type = Odometry
                    topic_callback = self.odometry_callback
                case "sensor_msgs/msg/PointCloud2":
                    from sensor_msgs.msg import PointCloud2
                    topic_type = PointCloud2
                    topic_callback = self.point_cloud_callback
                case "sensor_msgs/msg/Image":
                    from sensor_msgs.msg import Image
                    topic_type = Image
                    topic_callback = self.image_callback
                case "sensor_msgs/msg/CompressedImage":
                    from sensor_msgs.msg import Image
                    topic_type = Image
                    topic_callback = self.image_callback
                case _:
                    print(f"topic {topic_name} not configured")
                    continue

            print(f"subbing topic: {topic_name}, type: {topic_type}")
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

    def odometry_callback(self, msg: Odometry):
        pose = msg.pose.pose
        position = pose.position
        #rotation = np.array(rotation)

        translation = np.array([position.x, position.y, position.z])

        base = np.copy(translation)
        tail = np.array([position.x + 0.1, position.y, position.z])

        arrow = rr.Arrows3D(origins=[base], vectors=[tail])
        #rr.log("odometry", arrow)
        #rr.log("odometry", 

        #rr.log("odometry", rr.Transform3D(translation=translation))
        rr.log("odometry", rr.Points3D([translation]))

 
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

        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)

        pts = structured_to_unstructured(pts)

        if "rgb" in [x.name for x in points.fields]:
            points.fields = [
                PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
                PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
                PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
            ]
            colors = point_cloud2.read_points(points, field_names=["r", "g", "b"], skip_nans=True)
            colors = structured_to_unstructured(colors)
            rr.log("depth_registered/point_cloud", rr.Points3D(pts, colors=colors))

        else:
            # Log points once rigidly under robot/camera/points. This is a robot-centric
            # view of the world.
            rr.log("depth_registered/point_cloud", rr.Points3D(pts))


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
