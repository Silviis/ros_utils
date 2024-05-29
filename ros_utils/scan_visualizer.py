import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import cmath
import math
from functools import partial


# USAGE EXAMPLE:
# ros2 run ros_utils scan_visualizer --ros-args -p scan_topic:=tb3_2/scan -p msg_count:=50
class ScanVisualizer(Node):

    def __init__(self):
        super().__init__("scan_visualizer")
        self.declare_parameter("scan_topic", "tb3_0/scan")
        self.declare_parameter("msg_count", 10)
        self.scans = []
        self.max_range = 0
        self.plot_started = False

        self.topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.msg_count = (
            self.get_parameter("msg_count").get_parameter_value().integer_value
        )

        # Create scan subscriber
        self.get_logger().info(f"Creating scan subscriber for topic: {self.topic}")

        self.scan_sub = self.create_subscription(
            LaserScan, self.topic, partial(self.scan_cb), qos_profile_sensor_data
        )
        self.scan_sub  # Prevent unused variable warning

    def scan_cb(self, msg):

        # collect data only for msg_count amount of messages
        if len(self.scans) < self.msg_count:
            self.scans.append(msg)
            if msg.range_max > self.max_range:
                self.max_range = msg.range_max
        elif self.plot_started is False:
            self.plot_scans()

    def plot_scans(self):
        self.get_logger().info(f"Plotting scans")
        self.plot_started = True
        x = []
        y = []
        intensities = []
        colors = []
        max_intensity = 0
        min_intensity = 0

        for scan_msg in self.scans:
            angle = scan_msg.angle_min
            for range in scan_msg.ranges:
                pt = cmath.rect(range, angle)
                angle += scan_msg.angle_increment
                x.append(pt.real)
                y.append(pt.imag)
            for intensity in scan_msg.intensities:
                # get max intensity for color gradient
                if intensity > max_intensity:
                    max_intensity = intensity
                intensities.append(intensity)
        for intensity in intensities:
            colors.append(num_to_rgb(intensity, max_val=max_intensity))

        self.get_logger().info(f"Max intensity: {max_intensity}")
        self.get_logger().info(f"Min intensity: {min_intensity}")
        self.get_logger().info(f"Max range: {self.max_range}")
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(x, y, c=colors, s=5.5)
        ax.scatter(0, 0, c="black", s=100)
        # ax.text(-1.0, 2, "From topic: /tb3_X/" + self.topic)
        # ax.text(-1.0, 1.0, f"N = {self.msg_count} scans = {360 * self.msg_count} points")
        plt.show()


def num_to_rgb(val, max_val):
    i = val * 255 / max_val
    r = (math.sin(0.024 * i + 0) * 127 + 128) / 255
    g = (math.sin(0.024 * i + 2) * 127 + 128) / 255
    b = (math.sin(0.024 * i + 4) * 127 + 128) / 255
    return (r, g, b)


def main():
    rclpy.init()
    node = ScanVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
