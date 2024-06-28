from typing import Set
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.time import Time

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import copy
import matplotlib

import rerun as rr

from rerun_visualization.urdf import make_urdf_logger

from tf2_ros import TransformException, LookupException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import laser_geometry
from sensor_msgs_py import point_cloud2


class LidarVisualizationOption(Enum):
    Lines = 1
    Colour = 2

class RerunNode(Node):
    def __init__(self, lidar_visualization_option=LidarVisualizationOption.Colour):
        super().__init__("rerun")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.qos_profile = qos_profile

        self.joint_path_map = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.laser_proj = laser_geometry.laser_geometry.LaserProjection()
        self.lidar_visualization_option=lidar_visualization_option

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
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.laserscan_callback(x, topic_name)
                case "trajectory_msgs/msg/JointTrajectory":
                    from trajectory_msgs.msg import JointTrajectory

                    topic_type = JointTrajectory
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.joint_trajectory_callback(
                        x, topic_name
                    )
                case "nav_msgs/msg/Odometry":
                    from nav_msgs.msg import Odometry

                    topic_type = Odometry
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.odometry_callback(x, topic_name)
                case "sensor_msgs/msg/PointCloud2":
                    from sensor_msgs.msg import PointCloud2

                    topic_type = PointCloud2
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.point_cloud_callback(x, topic_name)
                case "sensor_msgs/msg/Image":
                    from sensor_msgs.msg import Image

                    topic_type = Image
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.image_callback(x, topic_name)
                case "sensor_msgs/msg/CompressedImage":
                    from sensor_msgs.msg import Image

                    topic_type = Image
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.image_callback(x, topic_name)
                case "nav_msgs/msg/OccupancyGrid":
                    from nav_msgs.msg import OccupancyGrid
                    
                    topic_type = OccupancyGrid
                    topic_callback = lambda x, topic_name=copy.deepcopy(topic_name): self.map_callback(x, topic_name)
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
        ranges = np.array(msg.ranges)
        lin_space = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        x_values_1 = np.cos(lin_space) * msg.range_min
        y_values_1 = np.sin(lin_space) * msg.range_min
        x_values_2 = np.cos(lin_space) * ranges
        y_values_2 = np.sin(lin_space) * ranges

        match self.lidar_visualization_option:
            case LidarVisualizationOption.Lines:
                line_points = np.vstack([y_values_1, x_values_1, y_values_2, x_values_2])
                lines = line_points.T.reshape(len(ranges), 2, 2)
                lines = rr.LineStrips2D(lines)

                rr.log(topic_name, lines)
            case LidarVisualizationOption.Colour: 
                cmap = matplotlib.colormaps["turbo_r"]
                norm = matplotlib.colors.Normalize(vmin=msg.range_min, vmax=msg.range_max)

                points = np.vstack((y_values_2, x_values_2)).T
                point_distances = np.linalg.norm(points, axis=1)
                point_colors = cmap(norm(point_distances))

                rr.log(topic_name, rr.Points2D(points, colors=point_colors))

    def odometry_callback(self, msg, topic_name):
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

        if "UC" in img.encoding or "FC" in img.encoding:
            return rr.log(topic_name, rr.DepthImage(cv_image))

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

        transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time(), timeout=Duration(seconds=10))

        for row in range(msg.info.height):
            for col in range(msg.info.width):
                value = msg.data[row * msg.info.width + col]
                if value == -1:
                    # image_data[row, col] = [255, 0, 0]  # Red for -1
                    image_data[row, col] = [0, 0, 0]
                else:
                    # Interpolate between black and white based on value
                    intensity = int((value / 100) * 255)
                    image_data[row, col] = [intensity, intensity, intensity]

        # Convert the occupancy grid image to grayscale
        gray_image = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection to the grayscale image
        edges = cv2.Canny(gray_image, 100, 200)

        # Apply Hough line transform to detect lines in the image
        # lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=80)

        # edged = image_data.copy()

        # # Draw the detected lines on the original image
        # if lines is not None:
        #     for line in lines:
        #         rho, theta, *_ = line[0]
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a * rho
        #         y0 = b * rho
        #         x1 = int(x0 + 1000 * (-b))
        #         y1 = int(y0 + 1000 * (a))
        #         x2 = int(x0 - 1000 * (-b))
        #         y2 = int(y0 - 1000 * (a))
        #         cv2.line(edged, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Apply Hough pobabilistic line transform to detect lines in the image
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60, minLineLength=10, maxLineGap=10)

        def distance_to_line(point, line):
            x1, y1, x2, y2 = line
            x0, y0 = point
            return np.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

        def distance_to_point(point1, point2):
            x1, y1 = point1
            x2, y2 = point2
            return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        LINE_MERGE_THRESHOLD = 20

        def lines_mergeable(line1, line2):
            x1, y1, x2, y2 = line1[0]
            x3, y3, x4, y4 = line2[0]

            slope1 = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else 0
            slope2 = (y4 - y3) / (x4 - x3) if x4 - x3 != 0 else 0
            return  (
                        abs(slope1 - slope2) < 0.5 or 
                        distance_to_line((x1, y1), (x3, y3, x4, y4)) < LINE_MERGE_THRESHOLD and
                        distance_to_line((x2, y2), (x3, y3, x4, y4)) < LINE_MERGE_THRESHOLD
                    ) \
                    and \
                    (
                        distance_to_point((x1, y1), (x3, y3)) < LINE_MERGE_THRESHOLD or
                        distance_to_point((x1, y1), (x4, y4)) < LINE_MERGE_THRESHOLD or
                        distance_to_point((x2, y2), (x3, y3)) < LINE_MERGE_THRESHOLD or
                        distance_to_point((x2, y2), (x4, y4)) < LINE_MERGE_THRESHOLD
                    )

        def merge_lines(line1, line2):
            x1, y1, x2, y2 = line1[0]
            x3, y3, x4, y4 = line2[0]
            all_points = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
            all_points.sort(key=lambda point: (point[0], point[1]))
            return [[all_points[0][0], all_points[0][1], all_points[-1][0], all_points[-1][1]]]

        filtered_lines = lines.tolist() if lines is not None else []
            
        # Modify the lines to go from left to right
        for i, line in enumerate(filtered_lines):
            x1, y1, x2, y2 = line[0]
            if x1 > x2:
                filtered_lines[i] = [[x2, y2, x1, y1]]

        # Sort the lines by length
        filtered_lines.sort(key=lambda line: distance_to_point((line[0][0], line[0][1]), (line[0][2], line[0][3])))

        for i in range(0, len(filtered_lines)):
            for j in range(i + 1, len(filtered_lines)):
                if j >= len(filtered_lines):
                    break
                line1 = filtered_lines[i]
                line2 = filtered_lines[j]
                if line1 is not line2:
                    x1, y1, x2, y2 = line1[0]
                    x3, y3, x4, y4 = line2[0]
                    if lines_mergeable(line1, line2):
                        filtered_lines[i] = merge_lines(line1, line2)
                        filtered_lines.remove(line2)
                        j -= 1

        edged = image_data.copy()

        # Draw the detected lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(edged, (x1, y1), (x2, y2), (0, 0, 255), 2)

        edged_filtered = image_data.copy()

        for line in filtered_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(edged_filtered, (x1, y1), (x2, y2), (255, 0, 0), 2)


        map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z])
        # Calculate the position of the robot in the occupancy grid
        robot_position = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        map_resolution = msg.info.resolution
        map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y, 0])
        map_position = (robot_position - map_origin) / map_resolution

        # Draw a circle on the occupancy grid image at the robot's position
        cv2.circle(image_data, (int(map_position[0]), int(map_position[1])), 5, (0, 255, 0), -1)
        cv2.circle(edged, (int(map_position[0]), int(map_position[1])), 5, (0, 255, 0), -1)
        cv2.circle(edged_filtered, (int(map_position[0]), int(map_position[1])), 5, (0, 255, 0), -1)

        rr.log(
            f"{topic_name}/occupancy",
            rr.Image(image_data),
        )

        rr.log(
            f"{topic_name}/occupancy_with_edges",
            rr.Image(edges),
        )

        rr.log(
            f"{topic_name}/occupancy_with_lines",
            rr.Image(edged),
        )

        rr.log(
            f"{topic_name}/occupancy_with_lines_filtered",
            rr.Image(edged_filtered),
        )

        def construct_mesh(lines):
            vertices = []
            indices = []
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Scale the line coordinates based on the map resolution
                x1 *= map_resolution
                y1 *= map_resolution
                x2 *= map_resolution
                y2 *= map_resolution

                x1 += map_origin[0]
                y1 += map_origin[1]
                x2 += map_origin[0]
                y2 += map_origin[1]

                # Add vertices
                vertices.append([x1, y1, 0])
                vertices.append([x1, y1, 0.25])
                vertices.append([x2, y2, 0])
                vertices.append([x2, y2, 0.25])
                # Add indices
                indices.append(len(vertices) - 4)
                indices.append(len(vertices) - 3)
                indices.append(len(vertices) - 2)
                indices.append(len(vertices) - 3)
                indices.append(len(vertices) - 2)
                indices.append(len(vertices) - 1)

            return rr.Mesh3D(
                vertex_positions=vertices,
                triangle_indices=indices
            )

        mesh = construct_mesh(filtered_lines)
        rr.log(
            f"{topic_name}/occupancy_mesh",
            mesh,
        )

        v_pos = [
            [-1, -1, 0],
            [1, -1, 0],
            [-1, 1, 0],
            [1, 1, 0]
        ]

        # Scale the vertex positions
        scaled_v_pos = [
            [
                p[0] * 8 * msg.info.width / msg.info.height + msg.info.origin.position.x * map_resolution,
                p[1] * 8 + msg.info.origin.position.y * map_resolution,
                p[2] * 8 + msg.info.origin.position.z * map_resolution,
            ]
        for p in v_pos]

        # Log the image_data on a quad
        rr.log(
            f"{topic_name}/occupancy_quad",
            rr.Mesh3D(
                vertex_positions=scaled_v_pos,
                triangle_indices=[
                    0, 1, 2,
                    1, 2, 3
                ],
                vertex_texcoords=[
                    [0, 0],
                    [1, 0],
                    [0, 1],
                    [1, 1]
                ],
                albedo_texture=image_data,
            )
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
    """This is just an example on how to use the RerunNode."""
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
    rerun_node = RerunNode(lidar_visualization_option=LidarVisualizationOption.Colour)
    rerun_node.load_urdf(
        "/home/ubuntu/mini_pupper_ros_urdf/mini_pupper_description/urdf/mini_pupper_description.urdf"
    )
    rerun_node.auto_subscribe(topics=topics_to_subscribe_to)

    rclpy.spin(rerun_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rerun_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
