import cv2
import numpy as np
from Detector import Detector
from CameraController import CameraController


class CalibrationPackage:
    def __init__(self, dimension_distance_list=[]):
        self.dimension_distance_list = dimension_distance_list  # width, height, distance

    def add_detection(self, rotatedRect, dist):
        dims = rotatedRect[1]
        width = max(dims)
        height = min(dims)

        self.dimension_distance_list.append((width, height, dist))
        return self


class HandrailFilter:

    def __init__(self):
        self.known_height = 3.5  # cm
        self.known_width = 25.5  # cm
        self.focal_length_width = -1
        self.focal_length_height = -1

    def calibrate_from_package(self, calibration_pckg):
        # [self.calibrate_from_detection(rect, dist) for rect, dist in calibration_pckg.detection_distances]
        focal_lengths_list = [(self.get_focal_length_from_detection_width(width, dist),
                               self.get_focal_length_from_detection_height(height, dist))
                              for width, height, dist in calibration_pckg.dimension_distance_list]

        self.focal_length_width = sum(pair[0] for pair in focal_lengths_list) / len(focal_lengths_list)
        self.focal_length_height = sum(pair[1] for pair in focal_lengths_list) / len(focal_lengths_list)

    def calibrate_from_detection(self, rotatedRect, known_distance):
        dims = rotatedRect[1]
        width = max(dims)
        height = min(dims)
        self.focal_length_width = self.get_focal_length_from_detection_width(width, known_distance)
        self.focal_length_height = self.get_focal_length_from_detection_height(height, known_distance)

    def get_focal_length_from_detection_height(self, height, known_distance):
        return height * known_distance / self.known_height

    def get_focal_length_from_detection_width(self, width, known_distance):
        return width * known_distance / self.known_width

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
        return [(rect, self.get_distance_from_detection(rect))
                for rect in self.remove_non_handrail_detections(rotatedRects)]


def test_distance_calculator():
    detector = Detector()
    handrail_filer = HandrailFilter()
    cal_img = cv2.imread("Calibration_Images/cal_1m_0degrees.jpg")
    detection = detector.get_rects_from_bgr(cal_img)[0]
    handrail_filer.calibrate_from_detection(detection, 100)

    test_img = cv2.imread("Calibration_Images/cal_80cm_45degrees.jpg")
    detection = detector.get_rects_from_bgr(test_img)[0]
    print(handrail_filer.get_distance_from_detection(detection))


# create a test with multiple handrails in frame
def test_cam_dist_calculator():
    detector = Detector()
    handrail_filer = HandrailFilter()
    cal_img = cv2.imread("Calibration_Images/cal_1m_0degrees.jpg")
    detection = detector.get_rects_from_bgr(cal_img)[0]
    handrail_filer.calibrate_from_detection(detection, 100)

    cam = CameraController()
    test_img = cam.get_image()
    detections = detector.get_rects_from_bgr(test_img)
    out = handrail_filer.get_valid_detections_and_distances_list(detections)
    for pair in out:
        box = cv2.boxPoints(pair[0])
        box = np.int0(box)
        cv2.drawContours(test_img, [box], 0, (0, 0, 255), 2)
        img = cv2.resize(test_img, (700, 500))
        cv2.imshow("original", test_img)
        cv2.waitKey(0)
        print(pair[1])


def test_package_calibration():
    detector = Detector()
    handrail_filer = HandrailFilter()
    cal_pckg = CalibrationPackage()
    cal1 = cv2.imread("Calibration_Images/cal_1m_0degrees.jpg")
    det1 = detector.get_rects_from_bgr(cal1)[0]
    cal2 = cv2.imread("Calibration_Images/cal_80cm_0degrees.jpg")
    det2 = detector.get_rects_from_bgr(cal1)[0]
    cal_pckg.add_detection(det1, 100).add_detection(det2, 80)
    handrail_filer.calibrate_from_package(cal_pckg)

    test_img = cv2.imread("Calibration_Images/cal_80cm_45degrees.jpg")
    detection = detector.get_rects_from_bgr(test_img)[0]
    print(handrail_filer.get_distance_from_detection(detection))


if __name__ == "__main__":
    # test_cam_dist_calculator()
    # test_distance_calculator()
    test_package_calibration()
