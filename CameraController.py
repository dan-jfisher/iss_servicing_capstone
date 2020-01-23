import cv2
import numpy as np


class CameraController:

    def __init__(self):
        self.camera = cv2.VideoCapture(0)

    def get_image(self):
        # check camera connection and then store the current frame
        if not self.camera.isOpened():
            print("Error interfacing with camera")
        return self.camera.read()[0]


class Detector:

    def __init__(self):
        self.color_lower_bound = np.array([110, 50, 50])  # example value
        self.color_upper_bound = np.array([130, 255, 255])  # example value

    def get_mask_bgr(self, bgr_img):
        # Guassian Blur is used to remove noise from the image
        bgr_image_blurred = cv2.GaussianBlur(bgr_img, (5, 5), 0)

        # Hue differences are best measured in the HSV color space
        hsv_image = cv2.cvtColor(bgr_image_blurred, cv2.COLOR_BGR2HSV)

        # return a binary mask that corresponds with the pixels that fall in the color range
        return cv2.inRange(hsv_image, self.color_lower_bound, self.color_upper_bound)

    def get_rects_from_mask(self, pixel_mask):
        # store a list of contours outlining the shapes leftover
        #       a contour is a list of points outlining a shape in a binary image
        contours, hierarchy = cv2.findContours(pixel_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # return a list of rectangles that best fit each contour
        return [cv2.minAreaRect(contour) for contour in contours if cv2.contourArea(contour) > 0]

    def get_rects_from_bgr(self, bgr_img):
        mask = self.get_mask_bgr(bgr_img)
        return self.get_rects_from_mask(mask)


def test_camera_to_rect_pipeline():
    camera = CameraController
    detector = Detector

    img = camera.get_image()
    detections = detector.get_rects_from_bgr(img)


def test_picture_to_rect_pipeline():
    detector = Detector()
    img = cv2.imread("/home/dan/Pictures/oneHandrail.jpg")
    detections = detector.get_rects_from_bgr(img)

    box = cv2.boxPoints(detections[0])
    box = np.int0(box)
    print(box)
    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
    cv2.imshow("original", img)
    cv2.waitKey(0)


if __name__ == "__main__":
    test_picture_to_rect_pipeline()

