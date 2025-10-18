import cv2
import numpy as np
import math

class SeritTakip:
    def __init__(self):
        pass

    def region_of_interest(self, img):
        height, width = img.shape[:2]
        mask = np.zeros_like(img)
        roi_top = int(height * 0.4)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, roi_top),
            (0, roi_top)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_img = cv2.bitwise_and(img, mask)
        return masked_img

    def get_lines(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 60, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        cropped = self.region_of_interest(edges)
        lines = cv2.HoughLinesP(cropped, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=100)
        return lines

    def slope_from_points(self, x1, y1, x2, y2):
        if x2 - x1 == 0:
            return 90.0
        return (y2 - y1) / (x2 - x1)

    def get_steering_decision(self, lines, frame_width, frame_height):
        if lines is None:
            return "Serit Bulunamadı"

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = self.slope_from_points(x1, y1, x2, y2)
            intercept = y1 - slope * x1

            if slope < -0.5:
                left_lines.append((slope, intercept))
            elif slope > 0.5:
                right_lines.append((slope, intercept))

        if len(left_lines) == 0 or len(right_lines) == 0:
            return "Serit"

        left_avg = np.average(left_lines, axis=0)
        right_avg = np.average(right_lines, axis=0)

        left_x = (frame_height - left_avg[1]) / left_avg[0]
        right_x = (frame_height - right_avg[1]) / right_avg[0]

        lane_center = (left_x + right_x) / 2
        frame_center = frame_width / 2
        delta = int(lane_center - frame_center)

        print(type(delta))  # <class 'int'> olmalı

        if delta < -40:
            return "Sol"
        elif delta > 40:
            return "Sag"
        else:
            return "Duz git"

