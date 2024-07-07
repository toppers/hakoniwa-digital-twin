#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def arGenerator():
    fileName = "ar.png"
    dpi = 300
    size_in_cm = 15
    size_in_inch = size_in_cm / 2.54
    pixel_size = int(size_in_inch * dpi)  # ピクセルサイズを計算
    # 0: ID番号，150x150ピクセル
    generator = aruco.generateImageMarker(dictionary, 0, pixel_size)
    cv2.imwrite(fileName, generator)
    img = cv2.imread(fileName)

arGenerator()
