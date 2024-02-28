from matplotlib.pyplot import axis
import numpy as np
from numpy import linalg as LA

class trajGenerator:
    def __init__(self, path):
        self.nodes = path
        self.TS = np.array([node.cost for node in self.nodes])
        self.yaw = 0
        self.heading = np.zeros(2)

    def get_des_state(self, t):

        if t > self.TS[-1]: t = self.TS[-1] - 0.001

        i = np.where(t >= self.TS)[0][-1]

        t = t - self.TS[i]
        xs = self.nodes[i+1].path
        j = int(t/0.1)
        state = xs[j]*0.02
        return state