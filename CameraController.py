import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


# Class for interfacing with the camera
class CameraController:

    # Initializes camera interface
    def __init__(self):
        self.camera = PiCamera()
        self.raw_capture = PiRGBArray(self.camera)
        
        time.sleep(0.1)


    # Returns the most recent image from the camera
    def get_image(self):
        # check camera connection and then store the current frame
        self.camera.capture(self.raw_capture, format="bgr")
        image = self.raw_capture.array
        # display the image on screen and wait for a keypress
        return image   



if __name__ == "__main__":
    camera = CameraController()
    camera.get_image()
