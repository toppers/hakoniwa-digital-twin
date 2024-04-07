import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
#from infra_sensor_position_estimator import InfraSensorPositionEstimater


from math import cos, sin, radians
import numpy as np
from numpy.linalg import lstsq

class InfraSensorPositionEstimater:
    def __init__(self):
        #LiDAR Params
        self.contact_max = 3.5
        self.sensor_pos_y = 0.0
        self.sensor_pos_x = 0.0
        self.offset_distance = 0.105
        self.offset_x = 0
        self.offset_y = 0
        self.base_degree = 0


    def analyze(self, degrees, values):
        deg = np.array(degrees)
        distance = np.array(values)
        A = np.vstack([deg**2, deg, np.ones(len(deg))]).T
        coefficients, _, _, _ = lstsq(A, distance, rcond=None)
        a, b, c = coefficients
        if a == 0:
            return 0, 0, False
        else:
            deg_vertex = -b / (2 * a)
        distance_vertex = a * deg_vertex**2 + b * deg_vertex + c

        radian_degree = radians(self.base_degree - deg_vertex)
        value = distance_vertex + self.offset_distance
        y = value * cos(radian_degree) + self.offset_y
        x = value * sin(radian_degree) + self.offset_x
        #print(f"(y, x): ({y}, {x})")
        return y, x, True

    
    def write_pos(self, zero=False):
        x = 0
        y = 0
        if zero:
            pass
        else:
            x = (self.sensor_pos_x - self.analyzed_x) 
            y = (self.sensor_pos_y - self.analyzed_y)
        #print(f"(ax, ay): ({self.analyzed_x }, {self.analyzed_y })")
        print(f"( x,  y): ({x}, {y})")

    def run(self, degrees, values):
        if len(degrees) > 0:
            self.analyzed_y, self.analyzed_x, result = self.analyze(degrees, values)
            self.write_pos(result == False)
        else:
            self.write_pos(True)

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
            if msg.intensities[i] > 2000.0 and msg.ranges[i] > 0.0:
                if msg.ranges[i] < self.filter_range:
                    degrees.append(i)
                    values.append(msg.ranges[i])
                    print(f"{i} {msg.ranges[i]} {msg.intensities[i]}")
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
