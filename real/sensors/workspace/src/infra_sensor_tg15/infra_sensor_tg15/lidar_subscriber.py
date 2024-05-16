import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
#from infra_sensor_position_estimator import InfraSensorPositionEstimater

from .pos_estimator import InfraSensorPositionEstimater
from .lidar_filter import InfraSensorLidarFilter
from .lidar_params import lidar_param_scan_count_max
from .lidar_params import lidar_param_segment_threshold
from .lidar_params import lidar_param_range_average_num
from .lidar_params import lidar_param_sensor_b_degree
from .lidar_params import lidar_param_sensor_mean_max
from .lidar_params import lidar_param_sensor_th_variance
from .lidar_params import lidar_param_sensor_t_radius
from .lidar_params import lidar_param_sensor_t_cv

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/RobotAvator_cmd_pos', 10)
        self.filter = InfraSensorLidarFilter(lidar_param_range_average_num)
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.estimater = InfraSensorPositionEstimater(
            b_degree = lidar_param_sensor_b_degree, 
            mean_max = lidar_param_sensor_mean_max, 
            th_variance = lidar_param_sensor_th_variance, 
            t_radius = lidar_param_sensor_t_radius, 
            t_cv = lidar_param_sensor_t_cv)
        self.subscription  # prevent unused variable warning
 

    def listener_callback(self, msg):
        degrees, values = self.filter.filter_ranges(msg.intensities, msg.ranges, range_min=msg.range_min, angle_increment=msg.angle_increment)
        x, y = self.estimater.run(degrees, values, scan_count_max=lidar_param_scan_count_max, value_threshold=lidar_param_segment_threshold)
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