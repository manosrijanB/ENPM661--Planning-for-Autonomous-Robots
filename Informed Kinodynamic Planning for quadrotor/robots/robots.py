import numpy as np
from math import dist
import matplotlib.pyplot as plt
from ruckig import *
import reeds_shepp

class Reeds_shepp:

    def __init__(self, acc_lim = 5):
        # Define the duration:1
        self.weights = np.array([1, 1, 2])
        self.bounds = [np.array([0, 0, -np.pi]), 
                       np.array([100, 100, np.pi])]

        self.step_size = 0.5
        self.rho = 5.8 
        self.distmax = 10

    def connect(self, xi, xf):
        qs = reeds_shepp.path_sample(xi, xf, self.rho, self.step_size)
        dist = reeds_shepp.path_length(xi, xf, self.rho)
        return dist, np.array(qs)[:, :3]
    
    def heuristic(self, x, start, goal):
        d1 = reeds_shepp.path_length(start, x, self.rho)
        d2 = reeds_shepp.path_length(x, goal, self.rho)
        return d1 + d2
    
    def steer(self, xi, xf):
        i = np.random.randint(5)
        self.distmax = [5, 7.5, 10, 12.5, 15][i]
        d1, path = self.connect(xi, xf)
        
        if d1 > self.distmax:
            N = int(self.distmax/d1*len(path[:, 0]))
            d1 = self.distmax
            path = path[:N, :]
        return d1, path
    
    def dist(self, xi, xf):
        return dist(xi[:2], xf[:2])
    
    def plot(self, ax, node, parent, scale):
        ax.plot(*(node.path[:, :2].T), "-g",zorder = 5)

    def plot_path(self, ax, path, scale):
        xs, ys = [], []
        for node in path:
            if node.path is None: 
                continue
            xs.append(node.path[:, 0]) 
            ys.append(node.path[:, 1])
        xs, ys = np.hstack(xs), np.hstack(ys)
        ax.plot(xs, ys, '-', color = (1, 0, 0, 0.8), zorder = 10, linewidth = 2)


class DoubleIntegrator2D:

    def __init__(self, vel_lim = 10, acc_lim = 5):

        self.weights = np.array([1, 1, 2, 2])
        self.bounds = [np.array([0, 0, -vel_lim, -vel_lim]), 
                       np.array([100, 100, vel_lim, vel_lim])]

        self.otg = Ruckig(2, 0.01)
        self.inp = InputParameter(2)
        self.inp.max_velocity = [10, 10]
        self.inp.max_acceleration = [5, 5]
        self.inp.max_jerk = [2, 2]
        self.Tmax = 3

    def connect(self, xi, xf):
        self.inp.current_position = xi[:2]
        self.inp.current_velocity = xi[2:]

        self.inp.target_position = xf[:2]
        self.inp.target_velocity = xf[2:]
       
        trajectory = Trajectory(2)
        result = self.otg.calculate(self.inp, trajectory)
        Tf = trajectory.duration
        return Tf, self.get_path(trajectory, Tf)
    
    def get_path(self, traj, Tf):
        ts = np.arange(0, Tf, 0.1)
        pos, vel = [], []
        for t in ts:
            new_pos, new_vel, new_acc = traj.at_time(t)
            vel.append(new_vel)
            pos.append(new_pos)
        path = np.c_[pos, vel]
        return path

    def heuristic(self, x, start, goal):
        T1, _ = self.connect(start, x)
        T2, _ = self.connect(x, goal)
        return T1 + T2
    
    def steer(self, xi, xf):
        i = np.random.randint(4)
        self.Tmax = [2, 3, 4, 5][i]
        T, path = self.connect(xi, xf)
        
        if T > self.Tmax:
            N = int(self.Tmax/T*len(path[:, 0]))
            T = self.Tmax
            path = path[:N, :]

        return T, path
    
    def dist(self, xi, xf):
        return dist(xi[:2], xf[:2])
    
    def plot(self, ax, node, parent, scale):
        ax.plot(*(node.path[:, :2].T), "-g",zorder = 5)

    def plot_path(self, ax, path, scale):
        xs, ys = [], []
        for node in path:
            if node.path is None: 
                continue
            xs.append(node.path[:, 0]) 
            ys.append(node.path[:, 1])
        xs, ys = np.hstack(xs), np.hstack(ys)
        ax.plot(xs, ys, '-', color = (1, 0, 0, 0.8), zorder = 10, linewidth = 2)


class Quadrotor_traj:

    def __init__(self, vel_lim = 10, acc_lim = 5):

        self.weights = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1])
        self.bounds = [np.array([0]*3 + [-vel_lim]*3 + [-acc_lim]*3), 
                       np.array([100]*3 + [vel_lim]*3 + [acc_lim]*3)]

        self.otg = Ruckig(3, 0.01)
        self.inp = InputParameter(3)
        self.inp.max_velocity = [40, 40, 40]
        self.inp.max_acceleration = [30, 30, 30]
        self.inp.max_jerk = [20, 20, 20]
        self.Tmax = 3

    def connect(self, xi, xf, bl = 1):
        self.inp.current_position = xi[:3]
        self.inp.current_velocity = xi[3:6]
        self.inp.current_acceleration = xi[6:]

        self.inp.target_position = xf[:3]
        self.inp.target_velocity = xf[3:6]
        self.inp.target_acceleration = xf[6:]
       
        trajectory = Trajectory(3)
        result = self.otg.calculate(self.inp, trajectory)
        Tf = trajectory.duration
        if bl == 0: return Tf
        return Tf, self.get_path(trajectory, Tf)
    
    def get_path(self, traj, Tf):
        ts = np.arange(0, Tf, 0.1)
        pos, vel, acc = [], [], []
        for t in ts:
            new_pos, new_vel, new_acc = traj.at_time(t)
            vel.append(new_vel)
            pos.append(new_pos)
            acc.append(new_acc)
        path = np.c_[pos, vel, acc]
        return path

    def heuristic(self, x, start, goal):
        T1 = self.connect(start, x, 0)
        T2 = self.connect(x, goal, 0)
        return T1 + T2
    
    def steer(self, xi, xf):
        i = np.random.randint(4)
        self.Tmax = [2, 3, 4, 5][i]
        T, path = self.connect(xi, xf)
        
        if T > self.Tmax:
            N = int(self.Tmax/T*len(path[:, 0]))
            T = self.Tmax
            path = path[:N, :]

        return T, path
    
    def dist(self, xi, xf):
        return dist(xi[:3], xf[:3])
    
    def plot(self, ax, node, parent, scale = 1):
        ax.plot(*(node.path[:, :3].T * scale), "-g",zorder = 5)

    def plot_path(self, ax, path, scale = 1):
        xs, ys, zs = [], [], []
        for node in path:
            if node.path is None: 
                continue
            xs.append(node.path[:, 0]*scale) 
            ys.append(node.path[:, 1]*scale)
            zs.append(node.path[:, 2]*scale)
        xs, ys, zs = np.hstack(xs), np.hstack(ys), np.hstack(zs)
        ax.plot(xs, ys, zs, '-', color = (1, 0, 0, 0.8), zorder = 10, linewidth = 2)
