import rclpy
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
#from infra_sensor_position_estimator import InfraSensorPositionEstimater


from math import cos, sin, radians
import numpy as np
from numpy.linalg import lstsq

class InfraSensorPositionEstimater:
    def __init__(self, publisher):
        self.publisher = publisher
        #LiDAR Params
        self.contact_max = 3.5
        self.sensor_pos_y = 0.0
        self.sensor_pos_x = 0.0
        self.offset_distance = 0.10
        self.offset_x = 0
        self.offset_y = 0
        self.base_degree = 0
        self.mean_maxlen = 10
        self.position_history_x = deque(maxlen=self.mean_maxlen)
        self.position_history_y = deque(maxlen=self.mean_maxlen)
        self.threshold = 2.0
        self.average_x = self.average_y = 0

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

    
    def analyze_circle(self, degrees, values):
        pos_x = []
        pos_y = []
        y = 0
        x = 0
        R = self.offset_distance
        for degree, value in zip(degrees, values):
            radian_degree = radians(self.base_degree - degree)
            pos_y.append(value * cos(radian_degree))
            pos_x.append(value * sin(radian_degree))
        x_data = np.array(pos_x)
        y_data = np.array(pos_y)
        A = np.vstack([x_data, y_data, np.ones(len(x_data))]).T
        B = -x_data**2 - y_data**2 -R**2
        params, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
        D, E, F = params
        h = -D / 2
        k = -E / 2
        #r = np.sqrt(h**2 + k**2 - F)

        #print(f"Center: ({h}, {k}), Radius: {R}")
        return k, h, True
    
    def filter_outliers(self, values, threshold=2):
        if len(values) < 2:
            return values
        mean_val = np.mean(values)
        std_val = np.std(values)
        filtered_values = [x for x in values if abs(x - mean_val) <= threshold * std_val]
        return filtered_values if len(filtered_values) > 0 else values
    
    def write_pos(self, zero=False):
        x = 0
        y = 0
        if zero:
            pass
        else:
            x = (self.sensor_pos_x - self.analyzed_x) 
            y = (self.sensor_pos_y - self.analyzed_y)
            self.position_history_x.append(x)
            self.position_history_y.append(y)
            filtered_x = self.filter_outliers(list(self.position_history_x), self.threshold)
            filtered_y = self.filter_outliers(list(self.position_history_y), self.threshold)
            self.average_x = np.mean(filtered_x)
            self.average_y = np.mean(filtered_y)
        #print(f"(ax, ay): ({self.analyzed_x }, {self.analyzed_y })")
        #print(f"( x,  y): ({self.average_x}, {self.average_y})")
        twist_msg = Twist()
        twist_msg.linear.x = self.average_x
        twist_msg.linear.y = self.average_y
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        
    def run(self, degrees, values):
        if len(degrees) >= 3:
            #self.analyzed_y, self.analyzed_x, result = self.analyze(degrees, values)
            self.analyzed_y, self.analyzed_x, result = self.analyze_circle(degrees, values)
            self.write_pos(result == False)
        else:
            self.write_pos(True)

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'real_pos', 10)
        self.threshold_intencity = 2000.0
        self.filter_range = 0.7
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.estimater = InfraSensorPositionEstimater(self.publisher_)
        self.subscription  # prevent unused variable warning
 
    def filter_ranges(self, msg):
        degrees = []
        values = []
        i = 0
        while i < 360:
            if msg.intensities[i] > self.threshold_intencity and msg.ranges[i] > 0.0:
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
