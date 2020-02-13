import cv2
import numpy as np
from Detector import Detector


class HandrailFilter:

    def __init__(self):
        self.known_distance = 100  # cm
        self.known_height = 3.5  # cm
        self.known_width = 25.5 # cm
        self.focal_length_width = -1
        self.focal_length_height = -1

    def calibrate_from_detection(self, rotatedRect):
        dims = rotatedRect[1]
        width = max(dims)
        height = min(dims)
        self.calibrate_from_detection_width(width)
        self.calibrate_from_detection_height(height)

    def calibrate_from_detection_height(self, height):
        self.focal_length_height = height * self.known_distance / self.known_height

    def calibrate_from_detection_width(self, width):
        self.focal_length_width = width * self.known_distance / self.known_width

    def get_distance_from_detection(self, rotatedRect):
        dims = rotatedRect[1]
        width = max(dims)
        height = min(dims)
        distance_h = self.get_distance_from_detection_height(height)
        distance_w = self.get_distance_from_detection_width(width)
        return min(distance_h, distance_w)

    def get_distance_from_detection_height(self, height):
        return self.known_height * self.focal_length_height / height

    def get_distance_from_detection_width(self, width):
        return self.known_width * self.focal_length_width / width

    def is_handrail(self, rotatedRect):
        dims = rotatedRect[1]
        width = max(dims)
        height = min(dims)
        return width / height >= 2  # placeholder comparison

    def remove_non_handrail_detections(self, rotatedRects):
        filtered_rects = [rect for rect in rotatedRects if self.is_handrail(rect)]
        return filtered_rects

    def get_valid_detections_and_distances_list(self, rotatedRects):
        filtered_rects = self.remove_non_handrail_detections(rotatedRects)
        return [(rect, self.get_distance_from_detection_width(rect)) for rect in filtered_rects]


def test_distance_calculator():
    detector = Detector()
    handrail_filer = HandrailFilter()
    cal_img = cv2.imread("Calibration_Images/cal_1m_0degrees.jpg")
    detection = detector.get_rects_from_bgr(cal_img)[0]
    handrail_filer.calibrate_from_detection(detection)

    test_img = cv2.imread("Calibration_Images/cal_80cm_45degrees.jpg")
    detection = detector.get_rects_from_bgr(test_img)[0]
    print(handrail_filer.get_distance_from_detection(detection))

# create a test with multiple handrails in frame


if __name__ == "__main__":
    test_distance_calculator()

