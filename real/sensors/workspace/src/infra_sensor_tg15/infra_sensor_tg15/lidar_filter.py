from collections import deque
import numpy as np
import math

from .lidar_params import lidar_param_range_threshold
from .lidar_params import lidar_param_intensities_threshold
from .lidar_params import lidar_param_range_value_constant

class InfraSensorLidarFilter:
    def __init__(self, avg_count):
        self.avg_count = avg_count
        self.data_accumulator = None

    def filter_ranges_for_sacn(self, intensities, ranges, angle_min, angle_increment):
        indexes = []
        degrees = []
        values = []
        
        num_points = len(ranges)  # ranges配列の長さを取得
        if self.data_accumulator is None:
            self.data_accumulator = {i: deque(maxlen=self.avg_count) for i in range(num_points)}

        #print(f"ange_min={ange_min}, angle_increment={angle_increment}")
        deg = angle_min
        for i in range(num_points):
            if ranges[i] > lidar_param_range_threshold and intensities[i] > lidar_param_intensities_threshold:
                self.data_accumulator[i].append(ranges[i] * lidar_param_range_value_constant)

            if len(self.data_accumulator[i]) > 0:
                v_value = np.mean(self.data_accumulator[i])
            else:
                v_value = 0.0  # デフォルト値を設定

            degrees.append(deg)
            values.append(v_value)
            indexes.append(i)
            deg += angle_increment  # 角度をインクリメント
        
        return indexes, degrees, values

    def filter_ranges(self, intensities, ranges, angle_min, angle_increment):
        indexes = []
        degrees = []
        values = []
        
        num_points = len(ranges)  # ranges配列の長さを取得

        #print(f"ange_min={ange_min}, angle_increment={angle_increment}")
        deg = angle_min
        for i in range(num_points):
            if ranges[i] > lidar_param_range_threshold and intensities[i] > lidar_param_intensities_threshold:
                degrees.append(deg)
                values.append(ranges[i] * lidar_param_range_value_constant)
                indexes.append(i)
            deg += angle_increment  # 角度をインクリメント
        
        return indexes, degrees, values