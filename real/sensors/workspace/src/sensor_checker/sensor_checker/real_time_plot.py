import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class RealTimePlot:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.data = []

    def update(self, frame):
        self.ax.clear()
        if len(self.data) > 10:
            self.ax.hist(self.data, bins=20, density=True)
            self.ax.set_xlabel('Distance')
            self.ax.set_ylabel('Probability')
            self.ax.set_title('LaserScan Data Distribution')

            # データの平均と標準偏差を計算して表示
            mean = np.mean(self.data)
            std_dev = np.std(self.data)
            self.ax.text(0.05, 0.95, f'Mean: {mean:.2f} m', transform=self.ax.transAxes, verticalalignment='top')
            self.ax.text(0.05, 0.90, f'Std Dev: {std_dev:.4f} m', transform=self.ax.transAxes, verticalalignment='top')
            self.ax.text(0.05, 0.85, f'Noise: {(100.0*(std_dev/mean)):.2f} %', transform=self.ax.transAxes, verticalalignment='top')
            self.ax.text(0.05, 0.80, f'Num: {len(self.data)}', transform=self.ax.transAxes, verticalalignment='top')

    def start(self):
        ani = FuncAnimation(self.fig, self.update, interval=1000)
        plt.show()

    def add_data(self, value):
        print(f"{len(self.data)} : value: {value}")
        self.data.append(value)
        if len(self.data) > 1000:
            self.data.pop(0)
