from robot_core.driver import RobotDriver
from kinematics.estimator import Estimator
from kinematics.controller import PurePursuitController
from kinematics.geometry import Circle, Lemniscate, Rectangle

def run():
    '''Shape movements.'''
    robot = RobotDriver()
    estimator = Estimator(robot)
    controller = PurePursuitController(estimator, robot)
    
    state = estimator.get_state()
    print("start state is: " + str(state["x"]) + " " + str(state["y"]) + " " + str(state["yaw"]))
    
    circle = Circle(200)
    path_circle, radius = circle.generate_points_and_radius(100)
    
    lem = Lemniscate(375)
    path_lem, radius = lem.generate_points_and_radius(300)
    
    rectangle = Rectangle(100, 150)
    path_rect = rectangle.generate_points(10)
    
    controller.follow_path(path_lem)
    # time.sleep(3)
    # controller.follow_path(path_lem)
    # time.sleep(3)
    # controller.follow_path(path_lem)
    
    # robot.drawing_rectange(20, 15, 150, 250)
    
    state = estimator.get_state()
    print("final state is: " + str(state["x"]) + " " + str(state["y"]) + " " + str(state["yaw"]))