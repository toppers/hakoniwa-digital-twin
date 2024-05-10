import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
#from infra_sensor_position_estimator import InfraSensorPositionEstimater

from .pos_estimator import InfraSensorPositionEstimater
from .lidar_filter import InfraSensorLidarFilter

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/RobotAvator_cmd_pos', 10)
        self.filter = InfraSensorLidarFilter(360, 1)
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.estimater = InfraSensorPositionEstimater(179, 10, 1.0, 0.1, 0.1)
        self.subscription  # prevent unused variable warning
 

    def listener_callback(self, msg):
        degrees, values = self.filter.filter_ranges(msg.intensities, msg.ranges)
        x, y = self.estimater.run(degrees, values, scan_count_max=10)
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LIDARSubscriber()
    lidar_subscriber.get_logger().info("InfraSensor UP")
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
