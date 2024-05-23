import sys
import numpy as np
import threading
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class LiDARPlotter(QMainWindow):
    def __init__(self, max_distance=15):
        super().__init__()
        self.setWindowTitle('2D LiDAR Point Cloud')
        self.setGeometry(100, 100, 800, 800)

        self.segments = []
        self.running = False
        self.lock = threading.Lock()
        self.max_distance = max_distance

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(-max_distance, max_distance)
        self.ax.set_ylim(-max_distance, max_distance)
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
        self.timer.start(200)

    def update_plot(self):
        with self.lock:
            if self.do_transaction:
                return
            self.ax.cla()
            self.ax.set_xlim(-self.max_distance, self.max_distance)
            self.ax.set_ylim(-self.max_distance, self.max_distance)
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
    plotter = LiDARPlotter(max_distance=2)
    plotter.show()

    plotter._run()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        plotter.stop()
        sys.exit()
