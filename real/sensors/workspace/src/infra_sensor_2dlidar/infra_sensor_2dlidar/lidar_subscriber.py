import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received scan: [angle_min: {:.2f}, angle_max: {:.2f}, range_min: {:.2f}, range_max: {:.2f}]'.format(
            msg.angle_min, msg.angle_max, msg.range_min, msg.range_max))

def main(args=None):
    print("hello world1")
    rclpy.init(args=args)
    lidar_subscriber = LIDARSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
