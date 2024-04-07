import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from infra_sensor_position_estimator import InfraSensorPositionEstimater

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
        self.filter_range = 0.5
        self.estimater = InfraSensorPositionEstimater()
        self.subscription  # prevent unused variable warning
 
    def filter_ranges(self, msg):
        degrees = []
        values = []
        i = 0
        while i < 360:
            if msg.intensities[i] > 0.0 and msg.ranges[i] > 0.0:
                if msg.ranges[i] < self.filter_range:
                    degrees.append(i)
                    values.append(msg.ranges[i])
                    #print(f"{i} {msg.ranges[i]} {msg.intensities[i]}")
            i = i + 1
        return degrees, values

    def listener_callback(self, msg):
        degrees, values = self.filter_ranges(msg)
        self.estimater.run(degrees, values)


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LIDARSubscriber()
    lidar_subscriber.get_logger().info("InfraSensor UP")
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
