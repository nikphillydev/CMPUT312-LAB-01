from robot_core.sensors import Sensors
from robot_core.driver import RobotDriver

def run():
    '''This task kills the motors in case of an emergency.'''
    driver = RobotDriver()
    sensor = Sensors()
    driver.turning(0,0)
    
    