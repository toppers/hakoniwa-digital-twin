import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
import threading
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal
from .lidar_filter import InfraSensorLidarFilter
from .pos_estimator import InfraSensorPositionEstimater
from .lidar_params import *


import numpy as np
import math

def euler_from_quaternion(x,y,z,w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class LIDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.publisher_ = self.create_publisher(Twist, '/TB3RoboAvatar_cmd_pos', 10)
        self.filter = InfraSensorLidarFilter(lidar_param_range_average_num)
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            LaserScan,
            ros_topic_name_scan,
            self.listener_callback,
            qos_profile)
        self.subscription_imu = self.create_subscription(
            Imu,
            ros_topic_name_imu,
            self.listener_imu,
            qos_profile)
        self.estimater = InfraSensorPositionEstimater(
            b_degree = lidar_param_sensor_b_degree, 
            mean_max = lidar_param_sensor_mean_max, 
            th_variance = lidar_param_sensor_th_variance, 
            t_radius = lidar_param_sensor_t_radius, 
            t_cv = lidar_param_sensor_t_cv)
        self.subscription  # prevent unused variable warning

        # フラグとロックの初期化
        self.scan_mode = False
        self.lock = threading.Lock()

    def listener_imu(self, msg: Imu):
        #print("imu:", msg.orientation)
        # Convert quaternion to Euler angles
        euler_angle = euler_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w            
        )
        self.angle_x = euler_angle[0]
        self.angle_y = euler_angle[1]
        self.angle_z = euler_angle[2]
        #print("angle_x:", self.angle_x)
        #print("angle_y:", self.angle_y)
        #print("angle_z:", self.angle_z)

    def listener_callback(self, msg):
        with self.lock:
            scan_mode = self.scan_mode

        if self.estimater.is_scan_mode():
            indexes, degrees, values = self.filter.filter_ranges_for_scan(msg.intensities, msg.ranges, angle_min=msg.angle_min, angle_increment=msg.angle_increment)
            x, y = self.estimater.run(indexes, degrees, values, scan_count_max=lidar_param_scan_count_max, value_threshold=lidar_param_segment_threshold)
        elif scan_mode:
            indexes, degrees, values = self.filter.filter_ranges(msg.intensities, msg.ranges, angle_min=msg.angle_min, angle_increment=msg.angle_increment)
            x, y = self.estimater.run(indexes, degrees, values, scan_count_max=lidar_param_scan_count_max, value_threshold=lidar_param_segment_threshold)
        else:
            x = 0.0
            y = 0.0

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = self.angle_x
        twist_msg.angular.y = self.angle_y
        twist_msg.angular.z = self.angle_z
        self.publisher_.publish(twist_msg)

    def set_scan_mode(self, mode):
        with self.lock:
            self.scan_mode = mode

class KeyboardMonitor(QThread):
    mode_changed = pyqtSignal(bool)

    def run(self):
        while True:
            user_input = input('Enter command (s: scan mode, p: processing mode): ')
            if user_input == 's':
                self.mode_changed.emit(True)
            elif user_input == 'p':
                self.mode_changed.emit(False)

def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    lidar_subscriber = LIDARSubscriber()
    lidar_subscriber.get_logger().info("InfraSensor UP")

    print("Now scanning environments..., please wait.")
    # スレッドを作成して spin を実行
    spin_thread = threading.Thread(target=spin_node, args=(lidar_subscriber,))
    spin_thread.start()

    # キーボードモニターを設定
    keyboard_monitor = KeyboardMonitor()
    keyboard_monitor.mode_changed.connect(lidar_subscriber.set_scan_mode)
    keyboard_monitor.start()

    # PyQt5のイベントループを実行
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()
        keyboard_monitor.terminate()
        lidar_subscriber.destroy_node()

if __name__ == '__main__':
    main()
