import numpy as np
from Planning import RRT, RRTStar, Map
import matplotlib.pyplot as plt
from robots.robots import *
from Quadrotor import QuadSim
from trajGen import trajGenerator
import mpl_toolkits.mplot3d.axes3d as Axes3D

np.random.seed(1)

# 3D boxes   lx, ly, lz, hx, hy, hz
obstacles = [[0, 60, 0, 50, 70, 100],
             [50, 60, 0, 100, 70, 45],
             [85, 60, 45, 100, 70, 100],
             [50, 60, 90, 85, 70, 100],
             [50, 20, 0, 100, 30, 100],
            #  [70, 50, 0, 80, 80, 100]
            ]

# limits on map dimensions
bounds = np.array([0, 100])
# create map with obstacles
map = Map(obstacles, bounds, dim = 3)

#plan a path from start to goal
start = np.array([60, 10, 20, 0, 0, 0, 0, 0, 0])
goal  = np.array([50, 90, 80, 0, 0, 0, 0, 0, 0])

robot = Quadrotor_traj()

#initialise RRT
sampler = ['random', 'rejection', 'hitandrun'][0]

rrt = RRTStar(robot, map, start, goal, sampler, goal_sample_rate = 0.05, max_iter = 500, goal_threshold = 10)

path, min_cost = rrt.plan()

print(min_cost)

rrt.draw_scene(path, scale = 1)

fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim((0,2))
ax.set_ylim((0,2))
ax.set_zlim((0,2))

#initialise simulation with given controller and trajectory
traj = trajGenerator(path)
Tmax = traj.TS[-1]
des_state = traj.get_des_state
sim = QuadSim(des_state, Tmax)

map.plotobs(ax, scale= 0.02)
#create a figure
sim.run(ax)