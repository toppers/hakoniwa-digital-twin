from collections import deque
from math import cos, sin, radians
import numpy as np
from numpy.linalg import lstsq
from scipy.optimize import least_squares

def residuals(circle, x, y):
    # 中心(h, k)と半径r
    h, k, r = circle
    return (x - h)**2 + (y - k)**2 - r**2

def fit_circle(x, y):
    # 初期推定値: 重心を中心とし、平均距離を半径とする
    x_m = np.mean(x)
    y_m = np.mean(y)
    initial_guess = [x_m, y_m, np.mean(np.sqrt((x - x_m)**2 + (y - y_m)**2))]
    
    # 最小二乗法で円をフィット
    result = least_squares(residuals, initial_guess, args=(x, y))
    
    h, k, r = result.x
    # 残差の計算
    residual_values = residuals([h, k, r], x, y)
    mae = np.mean(np.abs(residual_values))
    variance = np.var(residual_values)
    
    return h, k, r, mae, variance    

class InfraSensorPositionEstimater:
    def __init__(self, b_degree, mean_max, th_variance):
        #LiDAR Params
        self.contact_max = 3.5
        self.sensor_pos_y = 0.0
        self.sensor_pos_x = 0.0
        self.sensor_r = 0.0
        self.offset_distance = 0.10
        self.offset_x = 0
        self.offset_y = 0
        self.base_degree = b_degree
        self.mean_maxlen = mean_max
        self.position_history_x = deque(maxlen=self.mean_maxlen)
        self.position_history_y = deque(maxlen=self.mean_maxlen)
        self.position_history_r = deque(maxlen=self.mean_maxlen)
        self.threshold = th_variance
        self.average_x = self.average_y = self.average_r = 0

    def analyze_circle(self, degrees, values):
        pos_x = []
        pos_y = []
        for degree, value in zip(degrees, values):
            radian_degree = radians(self.base_degree - degree)
            pos_y.append(value * cos(radian_degree))
            pos_x.append(value * sin(radian_degree))
        x_data = np.array(pos_x)
        y_data = np.array(pos_y)
        h, k, r, m, v = fit_circle(x_data, y_data)
        print(f"Center: ({h}, {k}), Radius: {r} MAE: {m} Variance: {v}")
        return h, k, r, True

    def filter_outliers(self, values):
        if len(values) < 2:
            return values
        mean_val = np.mean(values)
        std_val = np.std(values)
        filtered_values = [x for x in values if abs(x - mean_val) <= self.threshold * std_val]
        return filtered_values if len(filtered_values) > 0 else values
    
    def write_pos(self, zero=False):
        x = 0
        y = 0
        if zero:
            pass
        else:
            x = (self.sensor_pos_x - self.analyzed_x) 
            y = (self.sensor_pos_y - self.analyzed_y)
            r = (self.sensor_r - self.analyzed_r)
            self.position_history_x.append(x)
            self.position_history_y.append(y)
            self.position_history_r.append(r)
            filtered_x = self.filter_outliers(list(self.position_history_x))
            filtered_y = self.filter_outliers(list(self.position_history_y))
            filtered_r = self.filter_outliers(list(self.position_history_r))
            self.average_x = np.mean(filtered_x)
            self.average_y = np.mean(filtered_y)
            self.average_r = np.mean(filtered_r)
        #print(f"(ax, ay): ({self.analyzed_x }, {self.analyzed_y })")
        print(f"( x,  y, r ): ({self.average_x}, {self.average_y}, {self.average_r} )")
        return self.average_x, self.average_y
        
    def run(self, degrees, values):
        if len(degrees) >= 3:
            #self.analyzed_y, self.analyzed_x, result = self.analyze(degrees, values)
            self.analyzed_y, self.analyzed_x, self.analyzed_r, result = self.analyze_circle(degrees, values)
            return self.write_pos(result == False)
        else:
            return self.write_pos(True)