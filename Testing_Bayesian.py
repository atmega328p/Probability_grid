import matplotlib.pyplot as plt
import numpy as np
from Bayesian import Bayesian
from Bresenham import bresenham
import math

"""
Create empty probability matrix
"""
grid_size = 60
prob_grid = np.array([[0.5 for col in range(grid_size)] for row in range(grid_size)])
#print(prob_grid)

""" 
Create one bresenham line for testing
"""
"""
line_x = np.ones(8)*4
line_x = line_x.astype(int)
line_y = np.arange(8)
bresenham_lines = list(zip(line_x, line_y))
"""

"""
Create 360 degrees circle perimiter of bresenham lines from a center
point
"""
grid = 8
grid = np.array([[0 for col in range(grid)] for row in range(grid)])

# Endpoints are int the robot coord sys
endPoints = []
# Bresenham lines are int the world coord sys
bresenhamLines = []
# Center the robot in the prob grid
robot_row = int(grid_size/2)
robot_col = int(grid_size/2)
laser_range = 10

for degree in range(0, 360, 1):
    radian = math.radians(degree)
    x1 = math.cos(radian)*laser_range
    y1 = math.sin(radian)*laser_range
    point = (x1, y1)
    endPoints.append(point)

# For each laser beam
for x in range(0, len(endPoints)):
    # Calculate its line by Bresenham's algorithm
    """
    DONT FORGET TO FUCK MAPS, SENSOR IN WORLD COORD (ROBOT + SENSOR)
    """
    bresenhamLine = list(bresenham(robot_row, robot_col, robot_row+endPoints[x][0],  robot_col+ endPoints[x][1]))
    bresenhamLines.append(bresenhamLine)

""" 
Sensor coords relative to robot 
Calcualte new probabilites for prob grid
"""
shit = Bayesian(prob_grid)
shit.print_prob_grid()
print()

for x in range(len(bresenhamLines)):
    shit.bayes_handler(bresenhamLines[x], robot_row, robot_col)

shit.print_prob_grid()

