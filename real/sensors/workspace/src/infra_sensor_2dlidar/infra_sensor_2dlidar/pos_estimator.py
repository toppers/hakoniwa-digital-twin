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
    def __init__(self, b_degree, mean_max, th_variance, t_radius, t_cv):
        #LiDAR Params
        self.sensor_pos_y = 0.0
        self.sensor_pos_x = 0.0
        self.sensor_r = 0.0
        self.base_degree = b_degree
        self.mean_maxlen = mean_max
        self.position_history_x = deque(maxlen=self.mean_maxlen)
        self.position_history_y = deque(maxlen=self.mean_maxlen)
        self.position_history_r = deque(maxlen=self.mean_maxlen)
        self.threshold = th_variance
        self.target_radius = t_radius # meter
        self.target_check_value = t_cv
        self.average_x = self.average_y = self.average_r = 0.0
        self.scan_history = {}
        self.scan_data = {}
        self.scan_count = 0

    def analyze_circle(self, degrees, values):
        pos_x = []
        pos_y = []
        for degree, value in zip(degrees, values):
            radian_degree = radians(self.base_degree - degree)
            pos_y.append(value * cos(radian_degree))
            pos_x.append(value * sin(radian_degree))
        x_data = np.array(pos_x)
        y_data = np.array(pos_y)
        h, k, r, mae, variance = fit_circle(x_data, y_data)
        # 半径がターゲットに近いか確認し、MAE と Variance が閾値以下か評価
        is_target_radius = abs(r - self.target_radius) <= self.target_radius * 0.1  # 10%の許容範囲内
        is_valid_circle = mae < self.target_check_value and variance < self.target_check_value**2
        
        print(f"Center: ({h}, {k}), Radius: {r}, MAE: {mae}, Variance: {variance}, V_radius: {is_target_radius} V_circle: {is_valid_circle}")
        return h, k, r, is_target_radius and is_valid_circle

    def filter_outliers(self, values):
        if len(values) < 2:
            return values
        mean_val = np.mean(values)
        std_val = np.std(values)
        filtered_values = [x for x in values if abs(x - mean_val) <= self.threshold * std_val]
        return filtered_values if len(filtered_values) > 0 else values
    
    def write_pos(self, result=None):
        x = 0.0
        y = 0.0
        if result is None:
            return self.average_x, self.average_y
        else:
            analyzed_x, analyzed_y, analyzed_r = result
            x = (self.sensor_pos_x - analyzed_x) 
            y = (self.sensor_pos_y - analyzed_y)
            r = (self.sensor_r - analyzed_r)
            self.position_history_x.append(x)
            self.position_history_y.append(y)
            self.position_history_r.append(r)
            filtered_x = self.filter_outliers(list(self.position_history_x))
            filtered_y = self.filter_outliers(list(self.position_history_y))
            filtered_r = self.filter_outliers(list(self.position_history_r))
            self.average_x = np.mean(filtered_x)
            self.average_y = np.mean(filtered_y)
            self.average_r = np.mean(filtered_r)
            print(f"( x,  y, r ): ({self.average_x}, {self.average_y}, {self.average_r} )")
        return self.average_x, self.average_y
        
    def get_segments(self, degrees, values, value_threshold):
        segments = []
        current_segment = []
        previous_degree = degrees[0] - 1  # 最初の要素より1小さい値で初期化
        previous_value = values[0]

        for degree, value in zip(degrees, values):
            if degree != previous_degree + 1 or abs(value - previous_value) > value_threshold:
                # 連続性が途切れたら新しいセグメントを開始
                if current_segment:
                    segments.append(current_segment)
                current_segment = []
            current_segment.append((degree, value))
            previous_degree = degree
            previous_value = value

        # 最後のセグメントを追加
        if current_segment:
            segments.append(current_segment)        
        
        return segments
    
    def _scan(self, degrees, values):
        for degree, value in zip(degrees, values):
            if degree not in self.scan_history:
                self.scan_history[degree] = deque(maxlen=self.mean_maxlen)
            self.scan_history[degree].append(value)

    def _finalize_scan(self):
        # 収集したデータから平均値を計算
        for degree, value_deque in self.scan_history.items():
            self.scan_data[degree] = np.mean(value_deque)
        print("Environment scan completed and data averaged.")

    def scan(self, degrees, values, scan_count_max = 100):
        if self.scan_count < scan_count_max:
            self._scan(degrees, values)
            self.scan_count += 1
            return False  # スキャンがまだ終了していないことを示す
        else:
            if not hasattr(self, 'finalized'):
                self._finalize_scan()
                self.finalized = True  # スキャン終了とデータ処理完了のフラグ
            return True  # スキャンが完了したことを示す

    def is_significant_change(self, degree, value, threshold=0.5):
        if value > 0 and abs(self.scan_data[degree] - value) > threshold:
            return True
        return False

    def run(self, degrees, values, scan_count_max, value_threshold=0.1):
        if not self.scan(degrees, values, scan_count_max):
            print("scanning: ", self.scan_count)
            return  self.write_pos(None)
        # スキャンが完了している場合のみ以下の分析を行う
        significant_segments = []
        if len(degrees) >= 3:
            segments = self.get_segments(degrees, values, value_threshold)
            for segment in segments:
                significant_data = [(deg, val) for deg, val in segment if self.is_significant_change(deg, val)]
                if significant_data:
                    significant_segments.append(significant_data)

            index = 0
            for seg in significant_segments:
                seg_degrees, seg_values = zip(*seg)
                if len(seg_degrees) >= 3:
                    analyzed_y, analyzed_x, analyzed_r, valid = self.analyze_circle(np.array(seg_degrees), np.array(seg_values))
                    if valid:
                        target_result = (analyzed_x, analyzed_y, analyzed_r)
                        for deg, val in seg:
                            print(f"VALID segment[{index}] ( {deg} {val} )")
                        print(f"Valid Circle Found: ({analyzed_x}, {analyzed_y}, {analyzed_r})")
                        return self.write_pos(target_result)
                    else:
                        for deg, val in seg:
                            print(f"INVALID segment[{index}] ( {deg} {val} )")

            return self.write_pos(None)
        else:
            return self.write_pos(None)