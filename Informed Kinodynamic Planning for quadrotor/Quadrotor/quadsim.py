import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque
from . import params

class QuadSim:
    def __init__(self,des_state,Tmax,
                 pos = None, attitude = [0,0,0],
                 animation_frequency = 50,
                 control_frequency = 100):

        self.t = 0
        self.Tmax = Tmax
        self.dt = 1/control_frequency
        self.animation_rate = 1/animation_frequency
        self.control_iterations = int(control_frequency / animation_frequency)

        self.des_state = des_state
        if pos is None: pos = des_state(0)[:3]
        self.pos_history = deque(maxlen=100)


    def control_loop(self):
        des_state = self.des_state(self.t)
        origin = des_state[0:3]
        rot = np.eye(3)
        wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
        quadBodyFrame = params.body_frame.T
        quadWorldFrame = wHb.dot(quadBodyFrame)
        self.t += 0.1
        return quadWorldFrame[0:3]

    def run(self,ax = None,save = False):
        self.init_plot(ax)
        while self.t < self.Tmax + 2:
            frame = self.control_loop()
            self.update_plot(frame)
            plt.pause(self.animation_rate)

    def init_plot(self,ax = None):
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((0,2))
            ax.set_ylim((0,2))
            ax.set_zlim((0,2))
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        self.lines = ax.get_lines()[-4:]

    def update_plot(self,frame):

        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]

        for line, line_data in zip(self.lines[:3], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(frame[:,4])
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])
