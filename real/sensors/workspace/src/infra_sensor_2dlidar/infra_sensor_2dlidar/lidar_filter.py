from collections import deque
import numpy as np
from scipy.signal import medfilt

def apply_median_filter(data, kernel_size):
    # data が deque の場合、先に NumPy 配列に変換
    if isinstance(data, deque):
        data = np.array(data)
    # カーネルサイズがデータ数を超えないように調整
    kernel_size = min(kernel_size, len(data))
    # カーネルサイズが奇数でなければ、奇数にする
    if kernel_size % 2 == 0:
        kernel_size += 1
    return medfilt(data, kernel_size=kernel_size)

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
                #v_value = np.mean(self.data_accumulator[i])
                filtered_value = apply_median_filter(self.data_accumulator[i], len(self.data_accumulator[i]))
                # 最後の要素を結果として使用（メディアンフィルタ後）
                v_value = filtered_value[-1]
                degrees.append(i)
                values.append(v_value)
        
        return degrees, values
