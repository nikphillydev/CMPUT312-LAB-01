import math
import time
from robot_core.driver import RobotDriver
from kinematics.estimator import Estimator
from kinematics.helper import sgn

STRAIGHTLINE_LEN = 250      # mm
LINEAR_SPEED = 90           # mm / s
TURN_ANGLE = 360            # deg
ANGULAR_SPEED = 60          # deg / s


def run():
    '''Straight line and angular error analysis.'''
    robot = RobotDriver()
    estimator = Estimator(robot)
    state = estimator.get_state()
    print("current state is { x: " + str(state["x"]) + ", y: " + str(state["y"]) + ", yaw: " + str(state["yaw"]))
    
    # Straight line error measurement
    print("robot driving in straight line...")
    robot.drive_straight(LINEAR_SPEED, STRAIGHTLINE_LEN)
    state = estimator.get_state()
    print("state after driving straight is { x: " + str(state["x"]) + ", y: " + str(state["y"]) + ", yaw: " + str(state["yaw"]))
    error = math.sqrt((state["x"]-0)**2 + (state["y"]-STRAIGHTLINE_LEN)**2)
    print("length of error is :" + str(error) + " mm")
    
    time.sleep(10)
    
    # Reset state
    print("resetting state...")
    estimator.reset_state()
    state = estimator.get_state()
    print("current state is { x: " + str(state["x"]) + ", y: " + str(state["y"]) + ", yaw: " + str(state["yaw"]))
    
    # Rotating error measurement
    print("robot rotating in place...")
    robot.rotation_on_spot(ANGULAR_SPEED, TURN_ANGLE)
    state = estimator.get_state()
    print("state after is { x: " + str(state["x"]) + ", y: " + str(state["y"]) + ", yaw: " + str(state["yaw"]))
    
    target_angle = 90
    current_angle = state["yaw"]
    turn_error = target_angle - current_angle
    # idk why this works but it works
    if turn_error > 180 or turn_error < -180 :
        turn_error = -1 * sgn(turn_error) * (360 - abs(turn_error))
        
    print("angle error is :" + str(turn_error) + " deg")







    



