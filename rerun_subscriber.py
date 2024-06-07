import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan

import numpy as np
import matplotlib

import rerun as rr

rr.init("pupper")
rr.connect("10.0.8.92:9876")

class RerunSubscriber(Node):

    def __init__(self):
        super().__init__('rerun')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.cmap = matplotlib.colormaps["turbo_r"]

    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        for i, val in enumerate(ranges):
            if val < msg.range_min:
                ranges[i] = msg.range_min
            elif val > msg.range_max:
                ranges[i] = msg.range_max

        lin_space = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x_values = np.cos(lin_space) * ranges
        y_values = np.sin(lin_space) * ranges

        norm = matplotlib.colors.Normalize(vmin=0, vmax=10)

        points = np.dstack((y_values, x_values))[0]
        point_distances = np.linalg.norm(points, axis=1)
        point_colors = self.cmap(norm(point_distances))

        rr.log("lidar", rr.Points2D(points, colors=point_colors))

        self.get_logger().info(f"min_angle:      {msg.angle_min}", once=True)
        self.get_logger().info(f"max_angle:      {msg.angle_max}", once=True)
        self.get_logger().info(f"angle_increment:{msg.angle_increment}", once=True)
        self.get_logger().info(f"time_increment: {msg.time_increment}", once=True)
        self.get_logger().info(f"scan_time:      {msg.scan_time}", once=True)
        self.get_logger().info(f"range_min:      {msg.range_min}", once=True)
        self.get_logger().info(f"range_max:      {msg.range_max}", once=True)
        self.get_logger().info(f"range_len:      {len(msg.ranges)}", once=True)


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


if __name__ == '__main__':
    main()
