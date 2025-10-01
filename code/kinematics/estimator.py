# Written by Nikolai Philipenko
# CMPUT 301 - Fall 2025

import time
import math
from threading import Thread
from robot_core.driver import RobotDriver
from kinematics.state import State
from robot_core.constants import AXLE_TRACK


class Estimator:
    """This class implements the dead reckoning estimator used to estimate current robot state."""
    def __init__(self, driver: RobotDriver):
        # Robot driver reference
        self.driver = driver
        
        # Current robot state
        self.state = State()
        
        # Trapezoidal integration
        self.time = time.time()
        self.prev_time = time.time()
        self.pos_x = 0              # integrated x velocity         (running value)
        self.pos_y = 0              # integrated y velocity         (running value)
        self.theta = math.pi / 2    # integrated angular velocity   (running value)
        self.prev_speed_x = 0
        self.prev_speed_y = 0
        self.prev_angular_speed = 0
        
        # Update state thread
        self.update_thread_handle = Thread(target=self.update_thread)
        self.update_thread_handle.start()
                
    def update_thread(self):
        while(True):         
            # 200 Hz state update 
            time.sleep(0.005)

            # Get motor speeds from robot driver
            left_speed = self.driver.get_left_motor_speed()
            right_speed = self.driver.get_right_motor_speed()
            
            # Robot kinematics
            speed = (right_speed + left_speed) / 2.0
            angular_speed = (right_speed - left_speed) / (AXLE_TRACK)
            
            # Perform integrations ...
            
            self.time = time.time()
            delta_time = self.time - self.prev_time
            self.prev_time = self.time
            
            self.theta += 0.5 * (angular_speed + self.prev_angular_speed) * delta_time
            self.prev_angular_speed = angular_speed
            
            speed_x = speed * math.cos(self.theta)
            self.pos_x += 0.5 * (speed_x + self.prev_speed_x) * delta_time
            self.prev_speed_x = speed_x
            
            speed_y = speed * math.sin(self.theta)
            self.pos_y += 0.5 * (speed_y + self.prev_speed_y) * delta_time
            self.prev_speed_y = speed_y
            
            # Update robot state
            self.state.update_state(self.pos_x, self.pos_y, self.theta, speed_x, speed_y, angular_speed)
                
    def get_state(self):
        '''Returns the current robot state.\n
            [ x (mm); y (mm); yaw (rad); x_vel (mm/s); y_vel (mm/s); angular_vel (rad/s)]
        '''
        return self.state.get_state()
    
    def reset_state(self):
        '''Reset the robot state frame.'''
        self.state.reset_state()
        
        # Reset integrations ...
        self.pos_x = 0
        self.pos_y = 0
        self.theta = math.pi / 2
    