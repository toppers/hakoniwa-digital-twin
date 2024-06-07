import sys
import numpy as np
import threading
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class LiDARPlotter(QMainWindow):
    def __init__(self, x_min, x_max, y_min, y_max):
        super().__init__()
        self.setWindowTitle('2D LiDAR Point Cloud')
        self.setGeometry(50, 50, 400, 400)

        self.segments = []
        self.running = False
        self.lock = threading.Lock()
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_title('2D LiDAR Point Cloud')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.grid(True)
        self.do_transaction = False

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        self.setCentralWidget(central_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

    def update_plot(self):
        with self.lock:
            if self.do_transaction:
                return
            self.ax.cla()
            self.ax.set_xlim(self.x_min, self.x_max)
            self.ax.set_ylim(self.y_min, self.y_max)
            self.ax.set_title('2D LiDAR Point Cloud')
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.grid(True)
            
            for i, (angles, distances) in enumerate(self.segments):
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                self.ax.scatter(x, y, s=1, label=f'Segment {i+1}')
            
            if self.segments:
                self.ax.legend()
            
            self.canvas.draw()

    def add_data(self, angles, distances):
        with self.lock:
            self.segments.append((angles, distances))

    def add_data_done(self):
        with self.lock:
            self.do_transaction = False

    def clear_data(self):
        with self.lock:
            self.do_transaction = True
            self.segments = []

    def _run(self):
        self.running = True
        threading.Thread(target=self._data_updater).start()

    def _data_updater(self):
        while self.running:
            time.sleep(1)
            self.clear_data()
            for _ in range(3):
                new_angles = np.linspace(-np.pi/2, np.pi/2, 181)
                new_distances = np.random.uniform(0.5, 15.0, 181)
                self.add_data(new_angles, new_distances)

    def stop(self):
        self.running = False

if __name__ == "__main__":
    app = QApplication(sys.argv)
    plotter = LiDARPlotter(x_min=-0.5, x_max=3, y_min=-1, y_max=1)
    plotter.show()

    plotter._run()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        plotter.stop()
        sys.exit()
