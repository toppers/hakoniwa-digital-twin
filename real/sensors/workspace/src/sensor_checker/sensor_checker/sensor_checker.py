import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
import threading
import sys
from .real_time_plot import RealTimePlot

global plotter
plotter = None

class SensorChecker(Node):
    def __init__(self):
        super().__init__('sensor_checker')
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            '/LiDAR2D_scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # フラグとロックの初期化
        self.scan_mode = False
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        global plotter
        print(f"len: {len(msg.ranges)}")
        if plotter is not None:
            plotter.add_data(msg.ranges[41])

        i = 0
        for v in msg.ranges:
            if v > 0.1 and v < 0.2:
                print(f"range[{i}]={v}")
            i = i + 1


def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    global plotter
    plotter = RealTimePlot()
    rclpy.init(args=args)
    lidar_subscriber = SensorChecker()
    lidar_subscriber.get_logger().info("SensorChecker UP")

    spin_thread = threading.Thread(target=spin_node, args=(lidar_subscriber,))
    spin_thread.start()

    plotter.start()

if __name__ == '__main__':
    main()
