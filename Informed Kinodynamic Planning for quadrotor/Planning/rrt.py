import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from .rrtutils import *


class RRT:
    def __init__(self, robot, map, start, goal,
                 goal_sample_rate = 0.05,
                 max_iter = 100, 
                 goal_threshold = 5):
                
        self.start = Node(start)
        self.goal = Node(goal)
        self.goal_sample_rate = goal_sample_rate
        self.goal_theshold = goal_threshold
        self.max_iter = max_iter
        self.dim = start.shape[0]
        self.tree = Rtree(self.dim, robot.weights)
        self.robot = robot
        self.map = map

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.tree.add(self.start)
        for i in range(self.max_iter):
            #Generate a random node (rnd_node)
            rnd_state = self.get_random_state()
            #Get nearest node (nearest_node)
            nearest_node = self.tree.nearest(rnd_state)
            #Get new node (new_node) by connecting
            new_node = self.steer(nearest_node, rnd_state)
            #If the path between new_node and the nearest node is not in collision
            if not self.map.collision(new_node.path):
              self.tree.add(new_node)
              # If the new_node is very close to the goal, connect it
              # directly to the goal and return the final path
              if self.robot.dist(new_node.p, self.goal.p) <= self.goal_theshold:
                    return self.backtrack(new_node)
        # cannot find path
        return None, np.inf

    def sample(self):
        # Sample random point inside boundaries
        lower, upper = self.robot.bounds
        return lower + np.random.rand(self.dim)*(upper - lower)
    
    def steer(self, from_node, to_state):
        cost, path = self.robot.steer(from_node.p, to_state)
        new_cost = from_node.cost + cost
        to_node = Node(path[-1], from_node, new_cost, path)
        return to_node

    def get_random_state(self):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > self.goal_sample_rate:
            rnd = self.sample()
        else:
            rnd = self.goal.p
        return rnd

    def backtrack(self, node):
        """Compute the final path from the goal node to the start node"""
        path = []
        cost = node.cost
        while node.parent:
          path.append(node)
          node = node.parent
        path.append(self.start)
        return path[::-1], cost

    def draw_graph(self,ax, scale = 1):
        '''plot the whole graph'''
        for node in self.tree.all():
            if node.parent:
                self.robot.plot(ax, node, node.parent, scale)

    def draw_path(self,ax,path, scale = 1):
        '''draw the path if available'''
        if path is None:
            print("path not available")
        else:
            print(path[0])
            self.robot.plot_path(ax, path, scale)

    def draw_scene(self,path = None,ax = None, scale = 1):
        '''draw the whole scene'''
        if ax is None:
            fig = plt.figure()
            if self.map.dim == 3:
                ax = Axes3D.Axes3D(fig)
            elif self.map.dim == 2:
                ax = plt.axes()
            else:
                print('cannot plot for current dimensions')
                return
        self.draw_graph(ax, scale)
        self.draw_path(ax, path, scale)
        self.map.plotobs(ax, scale)
        plt.show()
