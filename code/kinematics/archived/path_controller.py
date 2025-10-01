# kinematics/path_controller.py

from robot_core.driver import RobotDriver
from kinematics.estimator import Estimator
import time
import math
import robot_core.constants
from kinematics.geometry import Circle, Lemniscate, Shape, Rectangle, Straight_line
from kinematics.state import State

class Controller:
    
    def __init__(self, robot_driver: RobotDriver):
        self.robot = robot_driver
        self.estimator = Estimator(robot_driver)
        self.current_path = None
        self.current_index = 0
        self.tolerance = 50.0  # Position tolerance in mm
        self.default_speed = 10.0  # Default speed in mm/s

    def follow_path(self, frequency=5, path_points=[], path_radius=[]):
        '''Follow a path defined by points and optional radius values'''
        if not path_points:
            print("No path points provided!")
            return False

        print("Following path with " + str(len(path_points)) + " points at " + str(frequency) + "Hz")
        delta_time = 1.0 / frequency
        self.current_index = 0
        
        while self.current_index < len(path_points):
            # Get current state
            state = self.estimator.get_state()
            x, y = state["x"], state["y"]
            current_yaw = state["yaw"]
            
            target_x, target_y = path_points[self.current_index]

            print("Moving to point " + str(self.current_index + 1) + "/" + str(len(path_points)) + ": (" + str(target_x) + ", " + str(target_y) + ")")
            print("Current position: (" + str(x) + ", " + str(y) + ")")

            # Calculate distance to target
            distance_to_target = math.sqrt((target_x - x)**2 + (target_y - y)**2)


            
            self.default_speed = distance_to_target / delta_time
            # Calculate the angle to the target point
            target_angle = math.atan2(target_y - y, target_x - x)
            print("target_angle: " + str(target_angle))
            angle_error = target_angle - current_yaw

    
            
            # Normalize angle error to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            print("angle error:" + str(angle_error))
            
            omega = angle_error / delta_time
            # Use radius-based control if radius is provided and valid
            if (len(path_radius) > self.current_index and 
                path_radius[self.current_index] > 0):
                
                # Curved path control
                radius = path_radius[self.current_index]
                angular_speed = self.default_speed / radius  # rad/s
                
                # Apply angular speed in direction of angle error
                if abs(angle_error) > 0.1:
                    angular_speed *= math.copysign(1, angle_error)
                    print("  Turning: speed=" + str(self.default_speed) + ", angular_speed=" + str(angular_speed))
                    self.robot.turning_seconds(self.default_speed, omega, delta_time)
                else:
                    # Move straight if aligned
                    print("  Moving straight: speed=" + str(self.default_speed))
                    self.robot.turning_seconds(self.default_speed, 0, delta_time)
            else:
                # Point-to-point control
                if abs(angle_error) > 0.2:  # ~11 degrees
                    # Turn towards target first
                    turn_speed = 45.0  # degrees per second
                    turn_angle = math.degrees(angle_error)
                    print("  Rotating: " + str(turn_angle) + " degrees")
                    self.robot.rotation_on_spot(turn_speed, turn_angle)
                    # time.sleep(abs(turn_angle) / turn_speed + 0.2)
                else:
                    # Move straight towards target
                    move_distance = min(distance_to_target, self.default_speed * delta_time)
                    print("  Moving straight: " + str(move_distance) + "mm")
                    self.robot.drive_straight(self.default_speed, move_distance)
                    # time.sleep(move_distance / self.default_speed + 0.1)
            
            time.sleep(delta_time)
        
        print("Path following complete!")
        return True
        
    
    def follow_lemniscate(self, scaler, point_num=20, frequency=5):
        '''Follow a lemniscate path using the generated points and radius'''
        lemniscate = Lemniscate(scaler)
        path_points, path_radius = lemniscate.generate_points_and_radius(point_num)
        print("Generated lemniscate with " + str(len(path_points)) + " points")
        return self.follow_path(frequency, path_points, path_radius)
    
    
    def reset_estimator(self):
        '''Reset the estimator position'''
        self.estimator.reset_state()
        print("Estimator reset to origin")
    
    def get_current_position(self):
        '''Get current robot position'''
        state = self.estimator.get_state()
        return [state["x"], state["y"]]
    
    def set_tolerance(self, tolerance_mm):
        '''Set position tolerance for waypoint reaching'''
        self.tolerance = tolerance_mm
    
    def set_speed(self, speed_mm_per_s):
        '''Set default movement speed'''
        self.default_speed = speed_mm_per_s
