from grove.grove_mini_pir_motion_sensor import GroveMiniPIRMotionSensor
import time


class SensorController:
1
    def __init__(self):
        self.motion_sensor_pin = 24
        self.motion_sensor = GroveMiniPIRMotionSensor(self.motion_sensor_pin)
        def callback():
            print('Motion detected.')

        self.motion_sensor.on_detect = callback
        #  initialize all sensors here

if __name__=="__main__":
    sensor = SensorController()
    
    while True:
        print('No Motion detected.')
        time.sleep(1)