from robot_core.driver import RobotDriver
from kinematics.estimator import Estimator

def run():
    '''Dead reckoning controller.'''
    robot = RobotDriver()
    estimator = Estimator(robot)
    state = estimator.get_state()
    print("start state is: " + str(state["x"]) + " " + str(state["y"]) + " " + str(state["yaw"]))
    
    command = [
        [30, 40, 2],
        [-80, -60, 2],
        [20, 10, 2]
    ]

    for item in command:
        robot.wheel_power(item[0], item[1], item[2])
    state = estimator.get_state()
    print("final state is: " + str(state["x"]) + " " + str(state["y"]) + " " + str(state["yaw"]))
    