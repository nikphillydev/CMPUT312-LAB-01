# Written by Nikolai Philipenko and Yuyang Wang
# CMPUT 301 - Fall 2025

from ev3dev2.sensor import INPUT_1, INPUT_3
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor


class Sensors:
    """This class provides various data from onboard robot sensors."""
    def __init__(self):
        self.color = None
        self.distance = None
        self.angle = None
        self.lightstrengh = None
        
        # Sensors
        self.gyro = GyroSensor(INPUT_3)
        self.ultrasonic = UltrasonicSensor(INPUT_1)
        
        self.gyro.calibrate()
        self.gyro.reset()
            
    def get_distance_mm(self):
        '''Return the current distance measured from the ultrasonic sensor (mm)'''
        return self.ultrasonic.distance_centimeters * 10.0
    
    def get_angle_deg(self):
        '''Return the number of degrees that the gyroscope has been rotated (deg)'''
        return self.gyro.angle