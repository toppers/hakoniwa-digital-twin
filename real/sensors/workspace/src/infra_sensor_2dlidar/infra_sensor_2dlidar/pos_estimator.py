from collections import deque
from math import cos, sin, radians
import numpy as np
from numpy.linalg import lstsq

class InfraSensorPositionEstimater:
    def __init__(self):
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
        return self.average_x, self.average_y
        
    def run(self, degrees, values):
        if len(degrees) >= 3:
            #self.analyzed_y, self.analyzed_x, result = self.analyze(degrees, values)
            self.analyzed_y, self.analyzed_x, result = self.analyze_circle(degrees, values)
            return self.write_pos(result == False)
        else:
            return self.write_pos(True)