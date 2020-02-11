import cv2
import numpy as np
from Detector import Detector


class HandrailFilter:

    def __init__(self):
        self.known_distance = 100  # cm
        self.known_height = 3.5  # cm
        self.focal_length = -1

    def calibrate_from_detection(self, rotatedRect):
        self.focal_length = rotatedRect[1][1] * self.known_distance / self.known_height  # don't just assume [1][1], get the minimum

    def get_distance_from_detection(self, rotatedRect):
        return self.known_height * self.focal_length / rotatedRect[1][1]


def test_distance_calculator():
    detector = Detector()
    handrail_filer = HandrailFilter()
    cal_img = cv2.imread("Calibration_Images/cal_1m_0degrees.jpg")
    detection = detector.get_rects_from_bgr(cal_img)[0]
    handrail_filer.calibrate_from_detection(detection)

    test_img = cv2.imread("Calibration_Images/cal_80cm_45degrees.jpg")
    detection = detector.get_rects_from_bgr(test_img)[0]
    print(handrail_filer.get_distance_from_detection(detection))


if __name__ == "__main__":
    test_distance_calculator()

