# Written by Nikolai Philipenko
# CMPUT 312 - Fall 2025

import math
from threading import Lock


class State:
    """This thread-safe class defines the robot state used in dead reckoning. Object not to be instantiated directly in the application."""
    def __init__(self):
        self.state_mutex = Lock()
        self.x = 0              # mm
        self.y = 0              # mm
        self.yaw = math.pi / 2  # rad
        self.vel_x = 0          # mm/s
        self.vel_y = 0          # mm/s
        self.angular_vel = 0    # rad/s
    
    def update_state(self, new_x, new_y, new_yaw, new_vel_x, new_vel_y, new_angular_vel):
        '''Update the state vector with new values.'''
        self.state_mutex.acquire()
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw
        self.vel_x = new_vel_x
        self.vel_y = new_vel_y
        self.anglular_vel = new_angular_vel
        self.state_mutex.release()
        
    def reset_state(self):
        '''Reset the robot state to all zero (position and velocities).'''
        self.state_mutex.acquire()
        self.x = 0
        self.y = 0
        self.yaw = math.pi / 2
        self.vel_x = 0
        self.vel_y = 0
        self.angular_vel = 0
        self.state_mutex.release()
        
    def get_state(self):
        '''Returns the current robot state.\n
            [ x (mm); y (mm); yaw (rad); x_vel (mm/s); y_vel (mm/s); angular_vel (rad/s)]
        '''
        self.state_mutex.acquire()
        state_dict = {"x": self.x, "y": self.y, "yaw": self.yaw, "vel_x": self.vel_x, "vel_y": self.vel_y, "angular_vel": self.angular_vel }
        self.state_mutex.release()
        return state_dict