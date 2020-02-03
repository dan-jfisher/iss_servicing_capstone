from .grove.grove.grove_mini_pir_motion_sensor import *


class SensorController:

    def __init__(self):
        self.motion_sensor_pin = 0
        motion_sensor = GroveMiniPIRMotionSensor(self.motion_sensor_pin)
        #  initialize all sensors here
