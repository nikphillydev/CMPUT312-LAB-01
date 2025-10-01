import math
from typing import Tuple

# ------------- HELPER FUNCTIONS -------------

def pt_to_pt_distance (pt1: Tuple[float, float], pt2: Tuple[float, float]):
    '''Returns distance between pt1 (x1,y1) and pt2 (x2,y2).'''
    distance = math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    return distance

def sgn (num: float):
    '''Returns -1 if num is negative, 1 otherwise'''
    if num >= 0:
        return 1
    else:
        return -1