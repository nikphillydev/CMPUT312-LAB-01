# Written by Nikolai Philipenko and Yuyang Wang
# CMPUT 301 - Fall 2025

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, Motor, SpeedDPS
from robot_core.constants import *


class RobotDriver:
    """This class implements the low-level robot movement interface."""
    def __init__(self):
        self.left_motor = Motor(OUTPUT_B)
        self.right_motor = Motor(OUTPUT_A)
        self.tank_drive = MoveTank(OUTPUT_B, OUTPUT_A)

    def MMtoDegree(self, speed_in_mm):
        speed_in_degree = speed_in_mm / WHEEL_RADIUS * (180 / pi)
        return speed_in_degree
    
    def wheel_power(self, left_power: int, right_power: int, seconds: int):
        '''Apply a percentage of full power to the motors [0, 100] for some time in seconds.'''
        self.tank_drive.on_for_seconds(SpeedPercent(left_power), SpeedPercent(right_power), seconds)
    
    def drive_straight(self, speed, distance = 500.0):
        '''Drive the robot straight with a certain speed (mm/s) and distance (mm).'''
        self.tank_drive.on_for_rotations(SpeedDPS(self.MMtoDegree(speed)), SpeedDPS(self.MMtoDegree(speed)), distance / WHEEL_CIRCUMFERENCE)

    def rotation_on_spot(self, speed = 45.0, angle = 180.0):
        '''Turn the robot in place with a certain angular speed (dps) and angle (deg).'''
        turn_circumference = AXLE_TRACK * pi
        turn_distance = (angle / 360.0) * turn_circumference
        self.tank_drive.on_for_rotations(SpeedDPS(-speed * AXLE_TRACK / WHEEL_DIAMETER), SpeedDPS(speed * AXLE_TRACK / WHEEL_DIAMETER), turn_distance / WHEEL_CIRCUMFERENCE)

    def turning_seconds(self, speed, angular_speed, seconds):
        '''Drive the robot in a turn with a certain speed (mm/s) and angular speed (rps) for some time in seconds.'''
        self.tank_drive.on_for_seconds(SpeedDPS(self.MMtoDegree(speed - angular_speed * AXLE_TRACK / 2)) , SpeedDPS(self.MMtoDegree(speed + angular_speed * AXLE_TRACK / 2)), seconds)
    
    def turning(self, speed, angular_speed):
        '''Drive the robot in a turn with a certain speed (mm/s) and angular speed (rps) forever.'''
        self.tank_drive.on(SpeedDPS(self.MMtoDegree(speed - angular_speed * AXLE_TRACK / 2)) , SpeedDPS(self.MMtoDegree(speed + angular_speed * AXLE_TRACK / 2)))
    
    def driving(self, left_speed, right_speed, seconds = 2.0):
        '''For Task 4 and Braitenberg'''
        self.tank_drive.on_for_seconds(SpeedPercent(left_speed), SpeedPercent(right_speed), seconds)

    def drawing_rectange(self, lin_speed = 90.0, ang_speed = 45, length = 250.0, width = 200.0):
        '''Drive the robot in a rectangle with a certain linear speed (mm/s) and angular speed (deg/s), length (mm) and width (mm).'''
        for i in range(2):
            self.drive_straight(lin_speed, length)
            self.rotation_on_spot(ang_speed, 84.0)
            self.drive_straight(lin_speed, width)
            self.rotation_on_spot(ang_speed, 84.0)
        
    def get_left_motor_speed(self):
        '''Return left motor speed (mm/s).'''
        return self.left_motor.speed * (pi / 180.0) * WHEEL_RADIUS
    
    def get_right_motor_speed(self):
        '''Return right motor speed (mm/s).'''
        return self.right_motor.speed * (pi / 180.0) * WHEEL_RADIUS