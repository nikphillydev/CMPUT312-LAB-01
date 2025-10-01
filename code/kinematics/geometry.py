# Written by Yuyang Wang
# CMPUT 301 - Fall 2025

import math
from math import pi
import matplotlib.pyplot as plt


class Shape:
    """This class implements the shape drawer for polar coordinates to generate points of the shape."""
    def __init__(self, x_t, y_t, r_t, domain):
        '''x_t and y_t are the parametric equations of the shape'''
        self.x_t = x_t
        self.y_t = y_t
        self.r_t = r_t
        self.domain = domain

    def domain_points_creater(self, domain, point_num):
        '''domain is a 2d list that contains the start and end of each segment of the domain'''
        domain_points = []
        point_num_in_each_segment = point_num // len(domain)
        for i in range(len(domain)):
            length = domain[i][1] - domain[i][0]
            step = length / point_num_in_each_segment
            for j in range(point_num_in_each_segment):
                domain_points.append(domain[i][0] + j * step)
        return domain_points

    def shape_drawer(self, x_t, y_t, domain_points):
        '''domain is a list that contains a evenly spaced values in its domain
        domain should also contain the information of point number'''
        path_points = []
        for i in range(len(domain_points)):
            x = x_t(domain_points[i])
            y = y_t(domain_points[i])
            path_points.append([x, y])
        return path_points
    
    def get_radius(self, r_t, domain_points):
        radius = []
        for i in range(len(domain_points)):
            r = r_t(domain_points[i]+domain_points[i+1])/2 if i < len(domain_points)-1 else r_t(domain_points[i])
            radius.append(r)
        return radius
    
    def generate_points_and_radius(self, point_num):
        '''Generate points for the shape'''
        domain_points = self.domain_points_creater(self.domain, point_num)
        return self.shape_drawer(self.x_t, self.y_t, domain_points), self.get_radius(self.r_t, domain_points)
    

class Circle(Shape):
    def __init__(self, radius):
        self.radius = radius
        def x_func(theta):
            return self.radius * math.cos(theta) - radius
        def y_func(theta):
            return self.radius * math.sin(theta)
        def r_func(theta):
            return self.radius

        super().__init__(x_func, y_func, r_func, [[0, 2 * pi]])


class Lemniscate(Shape):
    def __init__(self, scaler):
        self.scaler = scaler
        def y_func(theta):
            cos_2theta = math.cos(2 * theta)
            if cos_2theta < 0:
                cos_2theta = 0
            return self.scaler * math.cos(theta) * math.sqrt(cos_2theta)
        def x_func(theta):
            cos_2theta = math.cos(2 * theta)
            if cos_2theta < 0:
                cos_2theta = 0
            return self.scaler * math.sin(theta) * math.sqrt(cos_2theta)
        def r_func(theta):
            cos_2theta = math.cos(2 * theta)
            if cos_2theta < 0:
                cos_2theta = 0
            return self.scaler * math.sqrt(cos_2theta)

        super().__init__(x_func, y_func, r_func, [[-pi / 4, pi / 4], [5 * pi / 4, 3 * pi / 4]])


class Rectangle(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def generate_points(self, point_num = 5):
        path_points = []
        path_points.append([0, 0])
        path_points.append([0, self.b])
        path_points.append([-self.a, self.b])
        path_points.append([-self.a, 0])
        path_points.append([0, 0])
        return path_points
    

class Straight_line(Shape):
    def __init__(self, len):
        self.len = len
    def generate_points(self, point_num = 2):
        path_points = []
        path_points.append([0, 0])
        for i in range(1, point_num - 1):
            path_points.append([0, i * self.len / (point_num - 1)])
        return path_points


# def main():
#     circle = Circle(200)
#     path, radius = (circle.generate_points_and_radius(20))
#     # for i in range(len(path)):
#     #     path[i][1] += 200
#     path1 = [path[i][0] for i in range(len(path))]
#     path2 = [path[i][1] for i in range(len(path))]
#     plt.plot(path1, path2, 'o')
#     plt.show()
#     print(path1)

# def main():
#     lem = Lemniscate(500)
#     path, radius = lem.generate_points_and_radius(200)
#     path1 = [path[i][0] for i in range(len(path))]
#     path2 = [path[i][1] for i in range(len(path))]
#     plt.plot(path1, path2, 'o')
#     plt.show()

def main():
    rectangle = Rectangle(150, 250)
    path = rectangle.generate_points(10)
    path1 = [path[i][0] for i in range(len(path))]
    path2 = [path[i][1] for i in range(len(path))]
    plt.plot(path1, path2, 'o')
    plt.show()

if __name__ == '__main__':
    main()