import copy
import heapq
## By Ayush Ghosh ID:20757522 for Assignment 2 Question 2 ECE 457A

maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0]]


def increment_cost(some_node):
    some_node[2] += 1

def move_up(agent):
    # This function checks whether it is possible to move to the square above the given node.
    # If moving up is possible, the position, the current cost and the current traversed path
    # for the given node is then updated.
    # Returns false if square above given node is invalid.
    node = copy.deepcopy(agent)
    if (node[0] + 1 <= 24):
        if (maze [node[0]+1][node[1]] == 0):
            node[0] = node[0]+1
            increment_cost(node)
            node[3].append([node[0], node[1]])
            return True, node

    return False, node

def move_right(agent):
    # This function checks whether it is possible to move to the square right of the given node.
    # If moving right is possible, the position, the current cost and the current traversed path
    # for the given node is then updated.
    # Returns false if square right of given node is invalid.
    node = copy.deepcopy(agent)
    if (node[1] + 1<= 24):
        if (maze [node[0]][node[1]+1] == 0):
            node[1] = node[1]+1
            increment_cost(node)
            node[3].append([node[0], node[1]])
            return True, node
    
    return False, node
    
def move_down(agent):
    # This function checks whether it is possible to move to the square below the given node.
    # If moving down is possible, the position, the current cost and the current traversed path
    # for the given node is then updated.
    # Returns false if square below given node is invalid.
    node = copy.deepcopy(agent)
    if (node[0] - 1 >= 0):
        if (maze [node[0]-1][node[1]] == 0):
            node[0] = node[0]-1
            increment_cost(node)
            node[3].append([node[0], node[1]])
            return True, node
    
    return False, node
    
def move_left(agent):
    # This function checks whether it is possible to move to the square left of the given node.
    # If moving up is possible, the position, the current cost and the current traversed path
    # for the given node is then updated.
    # Returns false if square left of given node is invalid.
    node = copy.deepcopy(agent)
    if (node[1] - 1 >= 0):
        if (maze [node[0]][node[1]-1] == 0):
            node[1] = node[1]-1
            increment_cost(node)
            node[3].append([node[0], node[1]])
            return True, node
    
    return False, node

def manhatten_heuristic(node, goal_index):
    # Calculates the manhatten heuristic between current position and goal position
    return (abs(node[0]- goal_index[0]) + abs(node[1]- goal_index[1])) 


def print_maze(list, end_goal):
    # Used to print output of the maze and traversed path
    maze_print = copy.deepcopy(maze)
    for i in range (25):
        for j in range(25):
            if maze_print[i][j] == 0:
                maze_print[i][j] = "   "
            if maze_print[i][j] == 1:
                maze_print[i][j] = " | "
    
    first_position = True
    for path_node in list:
        if first_position == True:
            maze_print[path_node[0]][path_node[1]] = " S "
            first_position = False
        elif path_node == end_goal:
            maze_print[path_node[0]][path_node[1]] = " E "
        else:
            maze_print[path_node[0]][path_node[1]] = " * "
            
    print(''.join('---' for word in range(25)))
    for i in range(25):
        print (''.join(str(word) for word in maze_print[24-i]))
    print(''.join('---' for word in range(25)))
    print(" ")

def bfs(start_pos, end_goal):
    # Node is defined in the MxN matrix as:
    # node = [M_pos, N_pos, cost, path_traversed[]]
    start_node  = [start_pos[0], start_pos[1], 1, []]
    start_node[3].append([start_node[0], start_node[1]])

    closed_list.append([start_node[0], start_node[1]])
    open_list.append(start_node)

    # intialize iteration counter
    count = 1

    while open_list:
        agent = copy.deepcopy(open_list.pop(0))
    
        if ([agent[0], agent[1]] == end_goal):

            print("Number of nodes explored including goal:", len(closed_list))
            print("Path cost:", agent[2])

            print ("Solution Path:")
            print (agent[3])
            
            print_maze(agent[3], end_goal)
            open_list.clear()
            closed_list.clear()
            break

        for i in range(4):
            status = False
            # Check each surrounding square in the order of up -> right -> down -> left
            if i == 0:
                [status, next_node] = move_up(agent)
            elif i == 1:
                [status, next_node] = move_right(agent)
            elif i == 2:
                [status, next_node] = move_down(agent)
            elif i == 3:
                [status, next_node] = move_left(agent)
            if status:  
                if [next_node[0], next_node[1]] not in closed_list:
                    closed_list.append([next_node[0], next_node[1]])
                    open_list.append(next_node)
            del next_node

        count += 1

def dfs(start_pos, end_goal):
    # Node is defined in the MxN matrix as:
    # node = [M_pos, N_pos, cost, path_traversed[]]
    start_node  = [start_pos[0], start_pos[1], 1, []]
    start_node[3].append([start_node[0], start_node[1]])

    closed_list.append([start_node[0], start_node[1]])
    open_list.append(start_node)

    # intialize iteration counter
    count = 1

    while open_list:
        agent = copy.deepcopy(open_list.pop(0))
    
        if ([agent[0], agent[1]] == end_goal):

            print("Number of nodes explored including goal:", len(closed_list))
            print("Path cost:", agent[2])

            print ("Solution Path:")
            print (agent[3])
            
            print_maze(agent[3], end_goal)
            open_list.clear()
            closed_list.clear()
            break

        for i in range(4):
            status = False

            # Check each surrounding square in the order of left -> down -> right -> up 
            # to account for inserting nodes at the correct order in front of the queue
            if i == 3:
                [status, next_node] = move_up(agent)
            elif i == 2:
                [status, next_node] = move_right(agent)
            elif i == 1:
                [status, next_node] = move_down(agent)
            elif i == 0:
                [status, next_node] = move_left(agent)
            if status:  
                if [next_node[0], next_node[1]] not in closed_list:
                    closed_list.append([next_node[0], next_node[1]])
                    open_list.insert(0, next_node)

            del next_node

        count += 1

def A_star(start_pos, end_goal):

    esitmated_cost = []
    heapq.heapify(esitmated_cost)
    # Node is defined in the MxN matrix as:
    # node = [M_pos, N_pos, cost, path_traversed[]]
    start_node  = [start_pos[0], start_pos[1], 1, []]
    start_node[3].append([start_node[0], start_node[1]])

    # f(n) = h(n) + g(n)
    f_value = manhatten_heuristic(start_node, end_goal) + start_node[2]
    
    
    closed_list.append([start_node[0], start_node[1]])
    heapq.heappush(esitmated_cost, [f_value, start_node])

    # intialize iteration counter
    count = 1
    
    while esitmated_cost:
        [f_value, get_node] = heapq.heappop(esitmated_cost)
        agent = copy.deepcopy(get_node)

        if ([agent[0], agent[1]] == end_goal):

            print("Number of nodes explored including goal:", len(closed_list))
            print("Path cost:", agent[2])

            print ("Solution Path:")
            print (agent[3])
            
            print_maze(agent[3], end_goal)
            
            open_list.clear()
            closed_list.clear()
            break

        for i in range(4):
            status = False
            # Check each surrounding square in the order of up -> right -> down -> left 
            if i == 0:
                [status, next_node] = move_up(agent)
            elif i == 1:
                [status, next_node] = move_right(agent)
            elif i == 2:
                [status, next_node] = move_down(agent)
            elif i == 3:
                [status, next_node] = move_left(agent)
            if status:  
                if [next_node[0], next_node[1]] not in closed_list:
                    
                    # f(n) = h(n) + g(n)
                    f_value = manhatten_heuristic(next_node, end_goal) + next_node[2]
                    
                    closed_list.append([next_node[0], next_node[1]])
                    heapq.heappush(esitmated_cost, [f_value, next_node])
        
            del next_node

        count += 1


# Initializing special states
S = [11, 2]
E1= [19, 23]
E2 = [21, 2]
bottom_left = [0, 0]
top_right = [24, 24]

closed_list = [] # Initializing a closed list
open_list = [] # Initializing an open list


## BFS, DFS, A* for Case S-E1
print("============")
print("Path S - E1:")
print("============")

print("BFS Output:")
bfs(S, E1)

print("DFS Output:")
dfs(S, E1)

print("A* Output:")
A_star(S, E1)

print(" ") # Spacer


## BFS, DFS, A* for Case S-E2
print("============")
print("Path S - E2:")
print("============")

print("BFS Output:")
bfs(S, E2)

print("DFS Output:")
dfs(S, E2)

print("A* Output:")
A_star(S, E2)

print(" ") # Spacer


## BFS, DFS, A* for Case (0,0) to (24,24)
print("========================")
print("Path (0, 0) to (24, 24):")
print("========================")

print("BFS Output:")
bfs(bottom_left, top_right)

print("DFS Output:")
dfs(bottom_left, top_right)

print("A* Output:")
A_star(bottom_left, top_right)
