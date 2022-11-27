import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.ndimage import rotate
import heapq 
from collections import deque as queue

# List containing directions for neighboring indices (i,j)
dirVal = [ (1,0), (0,1), (1,1), (1,-1), (-1, 1), (-1, -1), (-1,0), (0,-1)]

# Function to check if a cell to is be visited or not

def isLegal(maze, visited, nxt_pos):
   
    #Check if index lies is out of bounds
    if (nxt_pos[0] < 0 or nxt_pos[1] < 0 or nxt_pos[0] >= maze.shape[0] or nxt_pos[1] >= maze.shape[1]):
        return False
        
    #Check if index is visited
    if (visited[nxt_pos[0],nxt_pos[1]]):
        return False

    #Check if index is the maze wall
    if(maze[nxt_pos[0],nxt_pos[1]] == 1):
        return False
    return True

class node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  #Cost

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.position < other.position
        
def heuristic(x1,y1,x2,y2): # heuristic function based on the manhattan distance between the points x1,y1 x2,y2
    return abs(x1-x2) + abs(y1-y2) 

def astar(maze: np.ndarray, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = node(None, start)
    end_node = node(None, end)

    # Initialize both open and closed list
    open_list = []
    path = queue()
    visited = np.full(maze.shape, False)
    visited[start_node.position[0],start_node.position[1]] = True
    # Add the start node
    heapq.heappush(open_list, (0 ,(start_node, path))) # Priority queue using heappush, adding the starting node 
    
    # Loop until you find the end node

    while len(open_list) > 0:

        # Get t he current node
        item = heapq.heappop(open_list) 
        current_node = item[1][0]
        cost = current_node.g + 1
        p = item[1][1].copy()
        p.append(current_node.position)
        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node == end_node:
            # Complete here code to return the shortest path found
            return p

        # Complete here code to generate children, which are the neighboring nodes. You should use 4 or 8 points connectivity for a grid.

        for adj in dirVal:
            nxt_pos = tuple(np.add(current_node.position ,adj))
            if (isLegal(maze, visited, nxt_pos)):
                est_cost = cost + 1 + heuristic(nxt_pos[0], nxt_pos[1], end_node.position[0], end_node.position[0]) #estimated cost based on heuristic
                nxt_node = node(None, nxt_pos)
                nxt_node.g = cost
                heapq.heappush(open_list, (est_cost, (nxt_node, p)))
                visited[nxt_pos[0],nxt_pos[1]] = True
        

def find_path(start: tuple, goal: tuple, occupancy_grid):
    
    # Compute the path with A*
    path = np.array(list(astar(occupancy_grid, start, goal)))
    maze_plot=np.transpose(np.nonzero(occupancy_grid))

    plt.plot(maze_plot[:,0], maze_plot[:,1], '.',markersize=2)
    
    if not np.any(path): # If path is empty, will be NaN, check if path is NaN
        print("No path found")
    else:
        plt.plot(path[:,0], path[:,1], linewidth=3)
    plt.grid()
    plt.show()
