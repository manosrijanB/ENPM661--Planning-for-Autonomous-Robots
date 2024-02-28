import numpy as np
from Planning import RRT, RRTStar, Map
import matplotlib.pyplot as plt
from robots.robots import *

np.random.seed(1)

# #list of obstacles
obstacles = np.array([[30,30,60,50]])

#initialise environment map
bounds = np.array([0, 100])
map = Map(obstacles, bounds, dim = 2)

start = np.array([40, 10, 0, 0])
goal  = np.array([40, 70, 0, 0])

robot = DoubleIntegrator2D()

#initialise RRT
sampler = ['random', 'rejection', 'hitandrun'][2]

rrt = RRTStar(robot, map, start,goal, sampler, goal_sample_rate = 0.1, max_iter = 400)

path, min_cost = rrt.plan()

print(min_cost)

rrt.draw_scene(path)

