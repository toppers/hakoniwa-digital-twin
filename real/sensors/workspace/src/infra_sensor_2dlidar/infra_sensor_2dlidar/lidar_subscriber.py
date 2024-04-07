import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
 

    def listener_callback(self, msg):
        i = 0
        min = 10.0
        min_i = 0
        while i < 360:
            if msg.intensities[i] > 0.0 and msg.ranges[i] > 0.0:
                if msg.ranges[i] < 0.5:
                    print(f"{i} {msg.ranges[i]} {msg.intensities[i]}")
                    if min > msg.ranges[i]:
                        min = msg.ranges[i]
                        min_i = i
            i = i + 1
            print(f"MIN  {min_i} {min}")
        #self.get_logger().info(f'Received scan: [angle_min: {msg.angle_min:.2f}, angle_max: {msg.angle_max:.2f}, range_min: {msg.range_min:.2f}, range_max: {msg.range_max:.2f}, first_ten_ranges: [{ranges_str}]')


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LIDARSubscriber()
    lidar_subscriber.get_logger().info("InfraSensor UP")
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
