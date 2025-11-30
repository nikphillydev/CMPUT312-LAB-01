# Written by Nikolai Philipenko
# CMPUT 312 - Fall 2025

import math
import time
from typing import Tuple, List
from kinematics.helper import pt_to_pt_distance, sgn
from kinematics.estimator import Estimator
from robot_core.driver import RobotDriver


class PurePursuitController:
    """This class implements a Pure Pursuit controller to follow an arbitrary path, i.e. a list of (x,y) coordinates. Configurable parameters exist to achieve better results.
    """
    def __init__(self, estimator: Estimator, driver: RobotDriver):
        # Params
        self.look_ahead_distance = 30       # mm
        self.Kp_linear = 1.0
        self.Kp_turn = 1.2
        self.final_goal_tolerance = 10       # mm
        self.max_linear_speed = 33          # mm / s
        self.max_angular_speed = 60        # deg / s 
        self.x_solution_tolerance = 0.1     # mm
        self.y_solution_tolerance = 0.1     # mm
        self.update_rate = 200              # Hz
        
        self.debug = True
        
        # Estimator
        self.estimator = estimator
        # Motor driver
        self.driver = driver
        
        # Internal workings ...
        self.path = None
        self.last_found_index = 0
        self.approaching_final_position = False
        
    def follow_path(self, path: List[Tuple[float, float]]):
        '''Follow an arbitrary list of (x,y) coordinates. Returns once robot has traversed path and is within final_goal_tolerance of the last position.'''
        self.path = path
        self.last_found_index = 0
        self.approaching_final_position = False
        
        if (self.debug):
            print("STARTING PATH FOLLOWING")
        
        while (True):
            if (self.debug):
                print("STARTING FOLLOW PATH LOOP")
            
            # Get state
            state = self.estimator.get_state()
            current_pos = [state["x"], state["y"]]
            current_heading_rad = state["yaw"]
            current_heading_deg = (current_heading_rad * 180 / math.pi) % 360
            
            if (self.debug):
                print("CURRENT POSITION (mm): x: " + str(current_pos[0]) + ", y: " + str(current_pos[1]) + ", yaw (deg): " + str(current_heading_deg))
                print("last_found_index before search: " + str(self.last_found_index))
            
            # Get goal point on path that is look_ahead_distance from the robot
            goal_pt = self.get_goal_point(current_pos)
            
            if (self.debug):
                print("last_found_index after search: " + str(self.last_found_index))
                print("GOAL POINT FOUND (mm): x: " + str(goal_pt[0]) + ", y: " + str(goal_pt[1]))
            
            # Get velocities to move robot to goal point
            linear_vel, turn_vel = self.get_velocities(current_pos, current_heading_deg, goal_pt)
            
            if (self.debug):
                print("RAW  LINEAR AND ANGULAR VEL: (" + str(linear_vel) + " mm/s, " + str(turn_vel) + " deg/s)")
            
            # Normalized velocities to max / min bounds
            if (linear_vel > self.max_linear_speed): linear_vel = self.max_linear_speed
            if (turn_vel > self.max_angular_speed): turn_vel = self.max_angular_speed
            
            if (self.debug):
                print("NORM LINEAR AND ANGULAR VEL: (" + str(linear_vel) + " mm/s, " + str(turn_vel) + " deg/s)")
                print("COMMANDING MOTORS...")
                
            # Send velocites to robot motors for movement
            self.command_motors(linear_vel, turn_vel)
            
            # Check end condition
            if (self.approaching_final_position):
                if (pt_to_pt_distance(current_pos, goal_pt) < self.final_goal_tolerance):
                    if (self.debug):
                        print("PATH FOLLOWING COMPLETE. RESETTING STATE AND STOPPING MOTORS.")
                    # Reset internal state
                    self.path = None
                    self.last_found_index = 0
                    self.approaching_final_position = False
                    # Stop motors
                    self.command_motors(0, 0)
                    break
            
            # Sleep to allow robot to move ...
            time.sleep(1 / self.update_rate)
    
    def get_goal_point(self, current_pos: Tuple[float, float]):
        '''Return the point furthest on the path that is look_ahead_distance away (or closer) from the current_pos.'''
        # Check path validity
        if len(self.path) == 0:
            print("\tGGP: ERROR: path is empty, returning [null, null]")
            return [None, None]
        elif len(self.path) == 1:
            print("\tGGP: WARNING: path should have more than 1 point, returning this point.")
            self.approaching_final_position = True
            return [self.path[-1][0], self.path[-1][1]]
        
        # Extract current position coordinates
        currentX = current_pos[0]
        currentY = current_pos[1]

        # Initialize goal_pt in case no intersection is found
        goal_pt = [None, None]
        found_goal_pt = False
        
        # Path contains more than 1 point, find goal_pt using circle-line intersection ...
        
        for i in range (self.last_found_index, len(self.path) - 1):
            
            if (self.debug):
                print("\tGGP: ATTEMPTING TO FIND CIRCLE-LINE SOLUTION")
            
            # Extract points on the path that will create the line: (x1, y1) and (x2, y2)
            x1 = self.path[i][0]
            y1 = self.path[i][1]
            x2 = self.path[i + 1][0]
            y2 = self.path[i + 1][1]
            
            # Find points of intersection between circle centered at current_pos with 
            # radius look_ahead_distance and the infinite line drawn between (x1, y1) and (x2, y2)
            
            # Initialize solutions
            sol1 = [None, None]
            sol2 = [None, None]
            
            # Apply current_pos offset to "center" the circle at (0,0). This the simplifies the following math
            x1_offset = x1 - currentX
            y1_offset = y1 - currentY
            x2_offset = x2 - currentX
            y2_offset = y2 - currentY
            
            dx = x2_offset - x1_offset
            dy = y2_offset - y1_offset
            dr = math.sqrt(dx**2 + dy**2)
            D = x1_offset * y2_offset - x2_offset * y1_offset
            discriminant = self.look_ahead_distance**2 * dr**2 - D**2
            
            if (discriminant < 0):
                # Discriminant is negative, therefore no intersection exists between circle and line
                if (self.debug):
                    print("\tGGP: NO SOLUTION")
                found_goal_pt = False
                break
            else:
                # An intersection exists; it is either TANGENT (1 real point) or SECANT (2 real points)        

                x1_sol = (D * dy + sgn(dy) * dx * math.sqrt(discriminant)) / (dr * dr) + currentX   # adding back current_pos offset
                x2_sol = (D * dy - sgn(dy) * dx * math.sqrt(discriminant)) / (dr*dr) + currentX
                y1_sol = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (dr*dr) + currentY
                y2_sol = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (dr*dr) + currentY

                sol1 = [x1_sol, y1_sol]
                sol2 = [x2_sol, y2_sol]
                
                if (self.debug):
                    print("\tGGP: SOLUTIONS FOUND AT " + str(sol1) + " and " + str(sol2))
                    
                # Calculate min / max ranges to check if solutions are between (x1, y1) and (x2, y2)
                # Recall: math assumed infinite line going through (x1, y1) and (x2, y2)
                minX = min(x1, x2) - self.x_solution_tolerance
                maxX = max(x1, x2) + self.x_solution_tolerance
                minY = min(y1, y2) - self.y_solution_tolerance
                maxY = max(y1, y2) + self.y_solution_tolerance
                
                # Check if valid solution exists, i.e. a solution between (x1, y1) and (x2, y2)
                
                if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) or (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY):
                    
                    # At least one valid solution exists ...
                    
                    if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) and (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY):
                        # Both solutions are valid, check which one is closer to next point (x2, y2)
                        if (self.debug):
                            print("\tGGP: TWO VALID SOLUTIONS")
                            
                        if (pt_to_pt_distance(sol1, [x2,y2]) < pt_to_pt_distance(sol2, [x2,y2])):
                            if (self.debug):
                                print("\tGGP: USING FIRST SOLUTION")
                            goal_pt = sol1
                        else:
                            if (self.debug):
                                print("\tGGP: USING SECOND SOLUTION")
                            goal_pt = sol2
                            
                    else:
                        # Only one solution is valid
                        if (self.debug):
                            print("\tGGP: ONE VALID SOLUTION")
                            
                        if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY):
                            if (self.debug):
                                print("\tGGP: USING FIRST SOLUTION")
                            goal_pt = sol1
                        else:
                            if (self.debug):
                                print("\tGGP: USING SECOND SOLUTION")
                            goal_pt = sol2
                        
                    # Check if robot is closer to (x2, y2) than valid solution (i.e. the goal point). 
                    # In this case, update to the next line on path and re-run solution
                    
                    if (pt_to_pt_distance(current_pos, [x2, y2]) < pt_to_pt_distance(goal_pt, [x2, y2])):
                        if (self.debug):
                            print("\tGGP: ROBOT CLOSER TO LINE-ENDPOINT THAN GOAL POINT, INCREMENTING INDEX AND CONTINUE")
                        self.last_found_index = i + 1
                        continue
                    else:
                        # Found valid goal point
                        if (self.debug):
                            print("\tGGP: FOUND GOAL POINT")
                        found_goal_pt = True
                        self.last_found_index = i
                        break
                        
                else:
                    # Check if line is fully enclosed in circle
                    if (pt_to_pt_distance(current_pos, [x1, y1]) < self.look_ahead_distance and pt_to_pt_distance(current_pos, [x2, y2]) < self.look_ahead_distance):
                        if (self.debug):
                            print("\tGGP: LINE FULLY ENCLOSED IN CIRCLE, INCREMENTING INDEX AND CONTINUE")
                        self.last_found_index = i + 1
                        continue
                    
                    # No valid solutions exist within range
                    if (self.debug):
                        print("\tGGP: NO VALID SOLUTIONS")
                    found_goal_pt = False
                    break
        
        # Check if we are approaching final point on path
        if (self.last_found_index == (len(self.path) - 1)):
            if (self.debug):
                print("\tGGP: APPROACHING LAST POINT ON PATH")
            found_goal_pt = True
            self.approaching_final_position = True
            goal_pt = [self.path[-1][0], self.path[-1][1]] 
        
        if (not found_goal_pt):
            # No valid goal point on path within look_ahead_distance away from current_pos
            # Therefore go to last point seen on path
            if (self.debug):
                print("\tGGP: NO GOAL POINT FOUND, GOING TO LINE-STARTPOINT")
            goal_pt = [self.path[self.last_found_index][0], self.path[self.last_found_index][1]]
            
        return goal_pt
    
    def get_velocities (self, current_pos: Tuple[float, float], current_heading: float, target_pt: Tuple[float, float]) :
        '''Return the linear and angular velocities necessary to move the robot from the point: current_pos [x,y]
        with yaw: current_heading to the point: target_pt [x,y]. current_heading must be between [0, 360] (deg).'''
        
        # Get angle between current_pos and target_pt. Normalize between [0, 360] (deg)
        absTargetAngle = math.atan2(target_pt[1] - current_pos[1], target_pt[0] - current_pos[0]) * (180 / math.pi)
        if (absTargetAngle < 0): absTargetAngle += 360
        
        if (self.debug):
            print("\tGVEL: TARGET  ANGLE IS " + str(absTargetAngle) + " (deg)")
            print("\tGVEL: CURRENT ANGLE IS " + str(current_heading) + " (deg)")
        
        # Calculate errors between current and target
        linearError = math.sqrt((target_pt[0] - current_pos[0])**2 + (target_pt[1] - current_pos[1])**2)
        turn_error = absTargetAngle - current_heading  
        
        if (self.debug):
            print("\tGVEL: TURN ERROR BEFORE CORRECTION IS " + str(turn_error) + " (deg)")
        
        # idk why this works but it works
        if turn_error > 180 or turn_error < -180 :
            turn_error = -1 * sgn(turn_error) * (360 - abs(turn_error))  
            
        if (self.debug):
            print("\tGVEL: TURN ERROR AFTER  CORRECTION IS " + str(turn_error) + " (deg)")   
        
        # Convert errors into velocities with simple P controller
        linear_vel = linearError * self.Kp_linear
        turn_vel = turn_error * self.Kp_turn
    
        return linear_vel, turn_vel
    
    def command_motors(self, linear_vel: float, turn_vel: float):
        '''Command the robot to move with a certain linear velocity (mm/s) and angular velocity (deg/s).'''
        self.driver.turning(linear_vel, turn_vel * (math.pi / 180))