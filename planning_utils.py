from enum import Enum
from queue import PriorityQueue
import numpy as np
import utm


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
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    UP_LEFT = (-1, -1, np.sqrt(2))
    UP_RIGHT = (-1, 1, np.sqrt(2))
    DOWN_LEFT = (1, -1, np.sqrt(2))
    DOWN_RIGHT = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    #valid_actions = list(Action)
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN,Action.UP_LEFT, Action.DOWN_LEFT, Action.UP_RIGHT, Action.DOWN_RIGHT]
    
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)
    if y-1<0 or x-1<0 or grid[x-1,y-1] == 1:
        valid.remove(Action.UP_LEFT)
    if y-1<0 or x+1>n or grid[x+1,y-1] == 1:
        valid.remove(Action.DOWN_LEFT)
    if y + 1 > m or x-1<0 or grid[x-1,y+1] == 1:
        valid.remove(Action.UP_RIGHT)
    if y + 1 > m or x+1>n or grid[x+1,y+1] == 1:
        valid.remove(Action.DOWN_RIGHT)

    return valid


def a_star(grid, h, start, goal):

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
            for action in valid_actions(grid, current_node):
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

def global_to_local(global_position, global_home):
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])
    local_position = numpy.array([north - north_home, east - east_home, -(global_position[2] - global_home[2])])
    return local_position

def local_to_global(local_position, global_home):
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
                                                        global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1],
                               north_home + local_position[0], zone_number,
                               zone_letter)
    global_position = numpy.array([lon, lat, -(local_position[2]-global_home[2])])
    return global_position

def retrace_path( path, start):
    retrace_path = []
    pos = start
    for a in path:
        da = a#.value
        print(a,da,pos[0],da[0] )
        pos = (pos[0] + da[0], pos[1] + da[1])
        retrace_path.append(pos)
    return retrace_path

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
	
def bres(p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    cells = []
    dx,dy = x2-x1,y2-y1
    d=0
    i = x1
    j = y1
    while i<x2 and j<y2:
        cells.append([i,j])
        if d < dx - dy:
            d += dy
            i += 1
        elif d == dx - dy:
            d += dy
            i += 1
            d -= dx
            j += 1
        else:
            d -= dx
            j += 1
            
    return np.array(cells)

	
def path_free(grid, node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    cells = list(bres((x1, y1), (x2, y2)))
    
    for (x, y) in cells:
        if grid[x, y] > 0:
            return False
    
    return True

def expand_path(grid, start, goal, path, lookahead = 10, epochs = 1):
    """
    Prunes a path by expanding the distance between visible waypoints
    """
    search_path = path
    
    for epoch in range(epochs):
        new_path = [start]
        max_ind = len(search_path) - 1
        
        current_node = start
        index = 0
        
        while (index < max_ind):

            max_dist = 0

            for i in range(lookahead):
                new_index = index + i + 0

                if (new_index > max_ind):
                    break

                node = search_path[new_index]

                if (path_free(grid, current_node, node)):
                    dist = np.linalg.norm(np.array(current_node) - np.array(node))
                    if dist > max_dist:
                        max_dist = dist
                        index = new_index

            if (max_dist == 0):
                index += 1

            new_path.append(search_path[index])

            current_node = search_path[index]
        
        search_path = new_path
    
    return new_path