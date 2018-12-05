##############################################################################
# import packages
##############################################################################

import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure

##############################################################################
# plot grid
##############################################################################


# load the map file into numpy array
with open('map_data.txt', 'rt') as infile:
    grid1 = np.array([list(line.strip()) for line in infile.readlines()])
print('Grid shape', grid1.shape)

grid1[grid1 == '@'] = 1 #object on the map
grid1[grid1 == 'T'] = 1 #object on the map
grid1[grid1 == '.'] = 0 #free on map

grid = np.array(grid1.astype(np.int))
grid[grid == 1] = -1
route = []
num_of_agents = 3
num_of_error = 7
main_data = {}

####Plot The Map####
fig, ax = plt.subplots(figsize=(9, 5))
ax.imshow(grid, cmap=plt.cm.Dark2)


##############################################################################
# heuristic function for path scoring
##############################################################################

def heuristic(a, b):
    #return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    #Octile Distance
    #Heuristic Cost= (min(Differrence in x, Differrence in y) * square root of 2 + max(Differrence in x, Differrence in y) - min(Differrence in x, Differrence in y))

##############################################################################
# get start and goal
##############################################################################

def get_positions (lead_start, lead_goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    neighbor = random.choice(neighbors)

    new_start = lead_start[0] + neighbor[0], lead_start[1] + neighbor[1]
    new_goal = lead_goal[0] + neighbor[0], lead_goal[1] + neighbor[1]
    np.array(grid)

    return new_start, new_goal

##############################################################################
# path finding function
##############################################################################

def astar(array, start, goal, main_d, agent_num, err_num):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    ##############################################
    action_index = 0
    agent_num = ('agent' + repr(agent_num))
    pos_num = ('pos' + repr(action_index))
    main_d[agent_num] = {}

    # insert start point into main_data dict
    main_d[agent_num][pos_num] = (goal), err_num
    action_index = action_index + 1
    pos_num = ('pos' + repr(action_index))
    ##############################################


    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]

            #####################################
                #insert data into main_data dict
                main_d[agent_num][pos_num] = (current), err_num
                action_index = action_index + 1
                pos_num = ('pos' + repr(action_index))

            #####################################

            return grid, main_d

        close_set.add(current)
        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == -1:
                        # array bound -1 on map
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            #Check collision on the current move
            if array[neighbor[0]][neighbor[1]] == (action_index + 2):
                print('Collision between next step - ', array[neighbor[0]][neighbor[1]], 'and', (action_index+2))
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = (current)
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False

##############################################################################
# Get random start & goal
##############################################################################
secure_random = random.SystemRandom()
lead_start = (random.choice(np.argwhere(np.array(grid) == 0)))
lead_goal = (random.choice(np.argwhere(np.array(grid) == 0)))



for x in range(0, num_of_agents):

    #temp_path_data = {}
    #Get new Agent positions
    a_start, a_goal = get_positions(lead_start, lead_goal)



    start = (a_start[0], a_start[1])
    goal = (a_goal[0], a_goal[1])

    ##############################################################################
    # Calling A*
    ##############################################################################
    grid, main_data = astar(grid, start, goal, main_data, x, num_of_error)

    if main_data:
        print('########## Agent No. - ', x, '##########')
        print('Start point - ', a_start, 'Goal point - ', a_goal)
        print('Len of the route', len(main_data['agent' + repr(x)]))

    ##############################################################################
    # plot the path
    ##############################################################################

    # extract x and y coordinates from route list
    x_coords = []
    y_coords = []

    if main_data:
        for i in (range(0, len(main_data['agent' + repr(x)]))):

            x1 = main_data['agent' + repr(x)]['pos' + repr(i)][0][0]
            y1 = main_data['agent' + repr(x)]['pos' + repr(i)][0][1]

            grid[x1][y1] = i

            x_coords.append(x1)
            y_coords.append(y1)

    # plot path
    ax.scatter(start[1], start[0], marker="*", color="green", s=50)
    ax.scatter(goal[1], goal[0], marker="*", color="purple", s=50)
    ax.plot(y_coords, x_coords, color="yellow")

    start = []
    goal = []

plt.show()
