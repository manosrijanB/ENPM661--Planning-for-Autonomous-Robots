from rtree import index
import numpy as np
from matplotlib.pyplot import Rectangle

class Node:
    def __init__(self, state, parent = None, cost = np.inf, path = None):
        self.p = np.array(state)
        self.parent = parent
        self.cost = cost
        self.path = path

    def __len__(self):
        return len(self.p)

    def __getitem__(self, i):
        return self.p[i]

    def __repr__(self):
        return 'Node({}, {})'.format(self.p,self.cost)

#Rtree to store Nodes
class Rtree:
  def __init__(self, dim, S):
    self.dim = dim
    self.S = np.sqrt(S)
    self.node_list = []
    self.idx = self.get_tree(dim)
    self.len = 0

  @staticmethod
  def get_tree(dim):
    '''Initialise the tree'''
    p = index.Property()
    p.dimension = dim
    p.dat_extension = 'data'
    p.idx_extension = 'index'
    return index.Index(properties=p)

  def add(self,new_node):
    '''add nodes to tree'''
    self.node_list.append(new_node)
    self.idx.insert(self.len, new_node.p*self.S)
    self.len += 1

  def k_nearest(self,node,k):
    '''Returns k-nearest nodes to the given node'''
    near_ids = self.idx.nearest(node.p*self.S, k)
    for i in near_ids:
      yield self.node_list[i]

  def nearest(self, state):
    '''Returns nearest node to the given node'''
    near_ids = self.idx.nearest( state, 1)
    id = list(near_ids)[0]
    return self.node_list[id]

  def all(self):
    return self.node_list
