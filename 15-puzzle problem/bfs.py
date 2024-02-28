from queue import Queue
from time  import time
import numpy as np

# getting blank tile positions in i and j
def blanktile_pos(state):
    rs, cs = np.where(state == 0)
    i, j = rs[0], cs[0]
    return i, j 

#  action left
def actionmove_left(state):
    i, j  = blanktile_pos(state)
    # can't go left as it is left most corner
    if j == 0:
        return False, None
    else:
        # creating a copy inorder to overrwrite states
        state = state.copy()
        # interchanging 
        state[i, j], state[i, j-1] = state[i, j-1], state[i, j]
        
        return True, state
    
def actionmove_right(state):
    i, j  = blanktile_pos(state)
    #  can't go right as it is right most corner
    if j == 2:
        return False, None
    else:
        # creating a copy inorder to overrwrite  states
        state = state.copy()
        # interchanging 
        state[i, j], state[i, j+1] = state[i, j+1], state[i, j]
        
        return True, state

def actionmove_up(state):
    i, j  = blanktile_pos(state)
    #  can't go up as it in top
    if i == 0:
        return False, None
    else:
        # creating a copy inorder to overrwrite states
        state = state.copy()
        # interchanging 
        state[i, j], state[i-1, j] = state[i-1, j], state[i, j]
        
        return True, state    
    

def actionmove_down(state):
    i, j  = blanktile_pos(state)
    # can't go down as it is in down row
    if i == 2:
        return False, None
    else:
        # creating a copy inorder to overrwrite states
        state = state.copy()
        # interchanging 
        state[i, j], state[i+1, j] = state[i+1, j], state[i, j]
        
        return True, state


all_actions = [actionmove_left, actionmove_right, actionmove_up, actionmove_down]



# containing in a dictionary
def get_node(state, node_id, parent_id):
    Node  = {'state': state, 
             'parent_id': parent_id, 
             'node_id': node_id}
    return Node

# converting state array into string form
def state_to_str(state):
    flatten = state.T.flatten()
    return str(flatten)[1:-1]
    



# intial puzzle
start = np.array([[4,7,0],
                   [1,2,8],
                   [3,5,6]])

# start = np.array([[1,4,7],[5,0,8],[2,3,6]]) #test case 1
goal = np.array([[1,4,7 ],
                  [2,5,8],
                  [3,6,0]])


start_node = get_node(start, 0, None)

#  creating an array for the all the explored nodes
explored = [start_node]

# creating a function for nodes visited 
def is_visited(state):
    for node in explored:
        if (state == node['state']).all():
            return True
    return False
    
# creating a fucntion to check if goal is reached
def is_goal(state):
    return (state == goal).all()
    
# creating a queue to store the nodes
que = Queue()
que.put(start_node)
goal_node = None
# node index starting from zero
node_id = 0

st = time()
# if queue is not empty and goal node is none
while not que.empty() and goal_node is None:
    node = que.get()
# applyting all the actions
    for action in all_actions:
        # getting the status of state after the actions are done on them
        status, state = action(node['state'])
        if status is False: continue
        # if the node is not visited then continue to next 
        if not is_visited(state):
            #  adding new child nodes to the queue
            node_id += 1
            #  putting the values in a dictionary
            child_node = get_node(state, node_id, node['node_id'])
            # if goal node and child node are same then break the function
            if is_goal(state):
                goal_node = child_node
                break
            # then add the nodes to the explored
            explored.append(child_node)
            que.put(child_node)
et = time()
# time taken to run the while loop 
print(et- st)      
#  creating a empty list for back tracking path
path = []

curr_node = goal_node
# while start node is not reached
while curr_node['parent_id'] is not None:
    #  converting the array to string 
    state_str = state_to_str(curr_node['state'])
    path.append(state_str)
    # change current node to parent node
    
    idx = curr_node['parent_id']
    curr_node = explored[idx]
    
path = path[::-1]
# reversing the path from goal to start   
    
print(path)
#  creating a text file for nodespath
with open('nodesPath.txt', 'w+') as f:
    for line in path:
        f.write(line + '\n')
        
        
# creating a text file for nodes and nodes info
nodes = open('nodes.txt', 'w+')
nodesinfo = open('nodesinfo.txt', 'w+')
nodesinfo.write('nodes_id  parent_id\n')

for node in explored:
    line = state_to_str(node['state'])
    nodes.write(line + '\n')
    nodesinfo.write(str(node['node_id']) + ' ' + str(node['parent_id'])+'\n')

nodes.close()
nodesinfo.close()
        
        