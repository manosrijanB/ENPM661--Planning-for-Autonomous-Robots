from .rrt import *
from .sampleutils import Rejection, HitAndRun, Nonesampler
from math import ceil, dist

class RRTStar(RRT):

    def __init__(self, robot, map, start, goal, sampler,
                 goal_sample_rate = 0.1,
                 max_iter = 200,
                 goal_threshold = 5):

        super().__init__(robot, map, start, goal, goal_sample_rate, 
                          max_iter, goal_threshold)

        self.final_nodes = []
        self.dim = start.shape[0]
        lower, upper = self.robot.bounds
        Sampler = {'random': Nonesampler, 'rejection': Rejection, 'hitandrun': HitAndRun}[sampler]
        self.Informedsampler = Sampler(self.dim, self.robot.heuristic, start, goal, lower, upper)
 
    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.start.cost = 0
        self.tree.add(self.start)
        for i in range(self.max_iter):
            #Generate a random node (rnd_node)
            rnd_state = self.get_random_state()
            # Get nearest node
            nearest_node = self.tree.nearest(rnd_state)
            # Get new node by connecting rnd_node and nearest_node
            new_node = self.steer(nearest_node, rnd_state)
            # If path between new_node and nearest node is not in collision
            if not self.map.collision(new_node.path):
              #add the node to tree
              self.add(new_node)
        #Return path if it exists
        if not self.final_nodes: 
          return None, np.inf
        else: 
          return self.backtrack(self.goal)

    def add(self,new_node):
        near_nodes = self.near_nodes(new_node)
        # Connect the new node to the best parent in near_inds
        self.choose_parent(new_node, near_nodes)
        #add the new_node to tree
        self.tree.add(new_node)
        # Rewire the nodes in the proximity of new_node if it improves their costs
        self.rewire(new_node, near_nodes)
        #check if it is in close proximity to the goal
        if self.robot.dist(new_node.p, self.goal.p) <= self.goal_theshold:
            self.final_nodes.append(new_node)
        self.choose_parent(self.goal, self.final_nodes)


    def choose_parent(self, node, parents):
        """Set node.parent to the lowest resulting cost parent in parents and
           node.cost to the corresponding minimal cost
        """
        # Go through all near nodes and evaluate them as potential parent nodes
        for parent in parents:
          cost, path = self.robot.connect(parent.p, node.p)
          #checking whether a connection would result in a collision
          if not self.map.collision(path):
            #picking the parent resulting in the lowest cost and updating the cost of the new_node to the minimum cost.
            new_cost = parent.cost + cost
            if new_cost < node.cost:
              node.parent = parent
              node.cost = new_cost
              node.path = path

    def rewire(self, new_node, near_nodes):
        """Rewire near nodes to new_node if this will result in a lower cost"""
        #Go through all near nodes and check whether rewiring them to the new_node is useful
        for node in near_nodes:
          self.choose_parent(node,[new_node])
        self.propagate_cost_to_leaves(new_node, near_nodes)

    def near_nodes(self, node):
        """Find the nodes in close proximity to given node"""
        nnode = self.tree.len + 1
        r = ceil(5.5*np.log(nnode))
        return self.tree.k_nearest(node, r)

    def propagate_cost_to_leaves(self, parent_node, near_nodes):
        """Recursively update the cost of the nodes"""
        for node in near_nodes:
            if node.parent == parent_node:
                node.cost = self.new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node, near_nodes)

    def sample(self):
        """Sample random node inside the informed region"""
        lower, upper = self.robot.bounds
        if self.goal.parent:
          rnd = np.inf
          #sample until rnd is inside bounds of the map
          while True:
              # Sample random point inside informed region
              rnd = self.Informedsampler.sample(self.goal.cost)
              if not self.inbounds(rnd):
                self.Informedsampler.initialise()
              else: 
                break
        else:
          # Sample random point inside boundaries
          rnd = lower + np.random.rand(self.dim)*(upper - lower)
        return rnd
    
    def inbounds(self, p):
      '''Check if p lies inside map bounds'''
      lower, upper = self.robot.bounds
      return (lower <= p).all() and (p <= upper).all()
