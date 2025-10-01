# lab_tasks/task5.py
'''Braitenberg Vehicle'''
import time
import math
import sys
from robot_core.driver import RobotDriver
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2

left_sensor = ColorSensor(INPUT_1)
right_sensor = ColorSensor(INPUT_2)

MAX_SPEED = 80          # mm/s
MIN_SPEED = 40          # mm/s
MAX_ANGULAR_SPEED = math.pi / 2  # rad/s
FREQUENCY = 30          # Hz
SMOOTHING_FACTOR = 0.8 

def calibrate_sensors(sample_num=30, delay=0.2):
    print("move light source around sensors for calibration")
    print("Taking the " + str(sample_num) + " sample")
    
    left_readings = []
    right_readings = []
    
    for i in range(sample_num):
        left_val = left_sensor.ambient_light_intensity
        right_val = right_sensor.ambient_light_intensity
        left_readings.append(left_val)
        right_readings.append(right_val)

        print("Sample " + str(i+1) + ": L=" + str(left_val) + ", R=" + str(right_val))
        time.sleep(delay)
    
    l_min, l_max = min(left_readings), max(left_readings)
    r_min, r_max = min(right_readings), max(right_readings)
    
    print("Calibration Results:")
    print("Left sensor: " + str(l_min) + " - " + str(l_max) + " (range: " + str(l_max-l_min) + ")")
    print("Right sensor: " + str(r_min) + " - " + str(r_max) + " (range: " + str(r_max-r_min) + ")")

    if l_max - l_min < 10:
        print("WARNING: Left sensor range is very small!")
        sys.exit(1)
    if r_max - r_min < 10:
        print("WARNING: Right sensor range is very small!")
        sys.exit(1)

    return l_min, l_max, r_min, r_max

def normalize_sensor_value(value, min_val, max_val):
    if max_val == min_val:
        return 0.5
    else:
        return (value - min_val) / (max_val - min_val)

def get_normalized_sensors(l_min, l_max, r_min, r_max):
    left_raw = left_sensor.ambient_light_intensity
    right_raw = right_sensor.ambient_light_intensity
    
    left_norm = normalize_sensor_value(left_raw, l_min, l_max)
    right_norm = normalize_sensor_value(right_raw, r_min, r_max)
    
    return left_raw, right_raw, left_norm, right_norm

def Cowardice(robot: RobotDriver, l_min, l_max, r_min, r_max, duration=60):
    start_time = time.time()
    previous_angular_speed = 0.0
    
    '''turn away from light'''

    while time.time() - start_time < duration:
        left_raw, right_raw, left_norm, right_norm = get_normalized_sensors(l_min, l_max, r_min, r_max)
        print("raw: L=" + str(left_raw) + ", R=" + str(right_raw))

        total_light = (left_norm + right_norm) / 2
        total_light = max(0, min(1, total_light))
        print("total_light: " + str(total_light))

        base_speed = MIN_SPEED + total_light * (MAX_SPEED - MIN_SPEED)
        light_difference = right_norm - left_norm
        target_angular_speed = light_difference * MAX_ANGULAR_SPEED
        angular_speed = (SMOOTHING_FACTOR * target_angular_speed + (1 - SMOOTHING_FACTOR) * previous_angular_speed)
        previous_angular_speed = angular_speed
        
        robot.turning(base_speed, angular_speed)
        time.sleep(1.0 / FREQUENCY)
    
    robot.turning(0, 0)

def Aggression(robot: RobotDriver, l_min, l_max, r_min, r_max, duration=60):
    
    start_time = time.time()
    previous_angular_speed = 0.0
    
    '''turn towards light'''

    while time.time() - start_time < duration:
        left_raw, right_raw, left_norm, right_norm = get_normalized_sensors(l_min, l_max, r_min, r_max)
        print("raw: L=" + str(left_raw) + ", R=" + str(right_raw))
        total_light = (left_norm + right_norm) / 2
        total_light = max(0, min(1, total_light))
    
        base_speed = MIN_SPEED + total_light * (MAX_SPEED - MIN_SPEED) * 1.5
        
        light_difference = left_norm - right_norm
        print("turning " + str(light_difference))
        target_angular_speed = light_difference * MAX_ANGULAR_SPEED
        
        angular_speed = (SMOOTHING_FACTOR * target_angular_speed + 
                        (1 - SMOOTHING_FACTOR) * previous_angular_speed)
        previous_angular_speed = angular_speed
        
        robot.turning(base_speed, angular_speed)
        
        time.sleep(1.0 / FREQUENCY)
    
    robot.turning(0, 0)

def Love(robot: RobotDriver, l_min, l_max, r_min, r_max, duration=60):
    """
    Love - the vehicle to turn toward the light and circle it
    """
    start_time = time.time()
    previous_angular_speed = 0.0

    INHIBITION = 1.0
    SMALL_ORBIT_BIAS = 0.12 * MAX_ANGULAR_SPEED

    while time.time() - start_time < duration:
        left_raw, right_raw, left_norm, right_norm = get_normalized_sensors(l_min, l_max, r_min, r_max)
        print("raw: L=" + str(left_raw) + ", R=" + str(right_raw))

        left_norm = max(0.0, min(1.0, left_norm))
        right_norm = max(0.0, min(1.0, right_norm))

        # ipsilateral inhibition: when it gets brighter, the same side motor slows down
        left_motor = MIN_SPEED + (1.0 - INHIBITION * left_norm) * (MAX_SPEED - MIN_SPEED)
        right_motor = MIN_SPEED + (1.0 - INHIBITION * right_norm) * (MAX_SPEED - MIN_SPEED)

        left_motor = max(MIN_SPEED, min(MAX_SPEED, left_motor))
        right_motor = max(MIN_SPEED, min(MAX_SPEED, right_motor))

        base_speed = (left_motor + right_motor) / 2.0
        angular_speed = (right_motor - left_motor) / MAX_SPEED * MAX_ANGULAR_SPEED

        total_light = (left_norm + right_norm) / 2.0
        if abs(right_motor - left_motor) < 1e-3 and total_light > 0.9:
            bias_sign = math.copysign(1.0, previous_angular_speed) if abs(previous_angular_speed) > 1e-3 else 1.0
            angular_speed += bias_sign * SMALL_ORBIT_BIAS

        angular_speed = (SMOOTHING_FACTOR * angular_speed + (1 - SMOOTHING_FACTOR) * previous_angular_speed)
        previous_angular_speed = angular_speed

        robot.turning(base_speed, angular_speed)
        time.sleep(1.0 / FREQUENCY)

    robot.turning(0, 0)

def Curiosity(robot: RobotDriver, l_min, l_max, r_min, r_max, duration=60):
    
    start_time = time.time()
    previous_angular_speed = 0.0
    exploration_timer = 0
    exploration_direction = 1
    
    '''explore around light'''

    while time.time() - start_time < duration:
        left_raw, right_raw, left_norm, right_norm = get_normalized_sensors(l_min, l_max, r_min, r_max)
        print("raw: L=" + str(left_raw) + ", R=" + str(right_raw))
        total_light = (left_norm + right_norm) / 2
        total_light = max(0, min(1, total_light))
        
        base_speed = MIN_SPEED + total_light * (MAX_SPEED - MIN_SPEED) * 0.7
        
        exploration_timer += 1
        if exploration_timer > FREQUENCY * 3:
            exploration_direction *= -1
            exploration_timer = 0
        
        light_attraction = (right_norm - left_norm) * MAX_ANGULAR_SPEED * 0.5
        exploration_component = exploration_direction * MAX_ANGULAR_SPEED * 0.3
        
        target_angular_speed = light_attraction + exploration_component
        
        angular_speed = (SMOOTHING_FACTOR * target_angular_speed + (1 - SMOOTHING_FACTOR) * previous_angular_speed)
        previous_angular_speed = angular_speed
        
        robot.turning(base_speed, angular_speed)

        
        time.sleep(1.0 / FREQUENCY)
    
    robot.turning(0, 0)


def run():
    '''Braitenberg vehicle.'''
    robot = RobotDriver()
    l_min, l_max, r_min, r_max = calibrate_sensors()
    # Cowardice(robot, l_min, l_max, r_min, r_max, 20) 
    Aggression(robot, l_min, l_max, r_min, r_max, 20) 
    robot.turning(0, 0)


