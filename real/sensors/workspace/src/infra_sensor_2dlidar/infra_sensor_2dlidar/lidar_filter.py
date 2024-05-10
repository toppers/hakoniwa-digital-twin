from collections import deque
import numpy as np

class InfraSensorLidarFilter:
    def __init__(self, max_deg, avg_count):
        self.max_deg = max_deg
        self.avg_count = avg_count
        self.data_accumulator = {i: deque(maxlen=avg_count) for i in range(max_deg)}

    def filter_ranges(self, intensities, ranges):
        degrees = []
        values = []
        
        # データを蓄積し、平均を計算する
        for i in range(self.max_deg):
            self.data_accumulator[i].append(ranges[i])  # 新しいデータを追加
            
            # 平均値を計算（データが1つ以上あれば計算）
            if len(self.data_accumulator[i]) > 0:  
                avg_value = np.mean(self.data_accumulator[i])
                degrees.append(i)
                values.append(avg_value)
        
        return degrees, values
