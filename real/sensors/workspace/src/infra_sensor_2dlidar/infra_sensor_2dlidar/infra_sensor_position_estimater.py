#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import cos, sin, radians
import numpy as np
from numpy.linalg import lstsq

class InfraSensorPositionEstimater:
    def __init__(self):
        #LiDAR Params
        self.contact_max = 3.5
        self.sensor_pos_y = 0.0
        self.sensor_pos_x = 0.0
        self.offset_distance = 0.0
        self.offset_x = 0
        self.offset_y = 0
        self.base_degree = 0


    def analyze(self, degrees, values):
        deg = np.array(degrees)
        distance = np.array(values)
        A = np.vstack([deg**2, deg, np.ones(len(deg))]).T
        coefficients, _, _, _ = lstsq(A, distance, rcond=None)
        a, b, c = coefficients
        if a == 0:
            return 0, 0, False
        else:
            deg_vertex = -b / (2 * a)
        distance_vertex = a * deg_vertex**2 + b * deg_vertex + c

        radian_degree = radians(self.base_degree - deg_vertex)
        value = distance_vertex + self.offset_distance
        y = value * cos(radian_degree) + self.offset_y
        x = value * sin(radian_degree) + self.offset_x
        #print(f"(y, x): ({y}, {x})")
        return y, x, True

    
    def write_pos(self, zero=False):
        x = 0
        y = 0
        if zero:
            pass
        else:
            x = (self.sensor_pos_x - self.analyzed_x) / 100.0
            y = (self.sensor_pos_y - self.analyzed_y) / 100.0
        print(f"(ax, ay): ({self.analyzed_x }, {self.analyzed_y })")
        print(f"( x,  y): ({x}, {y})")

    def run(self, degrees, values):
        if len(degrees) > 0:
            self.analyzed_y, self.analyzed_x, result = self.analyze(degrees, values)
            self.write_pos(result == False)
        else:
            self.write_pos(True)

