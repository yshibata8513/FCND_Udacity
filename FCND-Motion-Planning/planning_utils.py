from enum import Enum
from queue import PriorityQueue
import numpy as np



def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

# Assume all actions cost the same.
class _Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    NORTH_EAST = (1, 1, np.sqrt(2))
    NORTH_WEST = (-1, 1, np.sqrt(2))
    SOUTH_EAST = (1, -1, np.sqrt(2))
    SOUTH_WEST = (-1, -1, np.sqrt(2))


    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def _valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(_Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    Neast = False
    Nwest = False
    Nsouth = False
    Nnorth = False

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(_Action.NORTH)
        Nnorth = True
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(_Action.SOUTH)
        Nsouth = True
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(_Action.WEST)
        Nwest = True
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(_Action.EAST)
        Neast = True

    #Neast(Nnorth,Nsouth,Nwest) means  
    # "valid_actions does not include Action.East(Nort,South,West)"
    if Neast or Nnorth or grid[x + 1, y + 1] == 1:
        valid_actions.remove(_Action.NORTH_EAST)
    if Nwest or Nnorth or grid[x - 1, y + 1] == 1:
        valid_actions.remove(_Action.NORTH_WEST)
    if Neast or Nsouth or grid[x + 1, y - 1] == 1:
        valid_actions.remove(_Action.SOUTH_EAST)
    if Nwest or Nsouth or grid[x - 1, y - 1] == 1:
        valid_actions.remove(_Action.SOUTH_WEST)

    return valid_actions


def _a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in _valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:            
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))



def check_coliniarity(x1,x2,x3,y1,y2,y3):
    det = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2)
    return det==0

def prune_pathpoints(path):
    _path = []
    _path.append(path[0])
    ind1 = 0
    ind2 = 1
    ind3 = 2
    while ind3<len(path):
        (x1,y1) = path[ind1][0],path[ind1][1]
        (x2,y2) = path[ind2][0],path[ind2][1]
        (x3,y3) = path[ind3][0],path[ind3][1]
        colin = check_coliniarity(x1,x2,x3,y1,y2,y3)
        if colin:
            ind2 = ind3
            ind3 += 1
        else:
            _path.append(path[ind2])
            ind1 = ind2
            ind2 = ind3
            ind3 += 1
    _path.append(path[-1])
    return _path
         
        

