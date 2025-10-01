# CMPUT312-LAB-01 Code

## Project Structure
The project was divided into three layers
* __Low-level layer - robot_core__: This layer interacts directly with ev3dev2 APIs and provides essential control functions for the robot's motors and sensors needed to operate the robot.
* __Middle-level layer - kinematics__: This layer handles all kinematic and geometric calculations, such as a state estimator for dead reckoning the current pose, functions to generate shape waypoints, and a well-designed pure-pursuit velocity controller.
* __High-level layer - lab_tasks__: This layer is used to implement tasks directly from our lab assignments.
```
CMPUT312-LAB-01/
├── main.py                    # main entrance of the program
├── robot_core/                # store all low-level robot control functions 
│   ├── __init__.py
│   ├── constants.py           # store constants like wheel radius, axle track
│   ├── driver.py              # encapsulate motor control functions
│   └── sensors.py             # encapsulate sensor reading functions
│
├── kinematics/                # store kinematics and geometry related functions
│   ├── __init__.py
│   ├── estimator.py           # dead reckoning calculations for robot position
│   ├── helper.py              # helper functions for our controller
│   ├── controller.py          # pure-pursuit controller for following shapes
│   ├── state.py               # managing the current state of the robot
│   └── geometry.py            # geometry of generating waypoints for shapes
│
└── lab_tasks/                 # store all lab task implementations
    ├── __init__.py
    ├── task2.py 
    ├── task3.py
    ├── task4.py
    └── task5.py
```

## Contributors
 - Nikolai Philipenko
 - Yuyang Wang
