from enum import Enum
from queue import PriorityQueue

import numpy as np

def create_voxmap(data, target_altitude, safety_distance, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data and safety distance

    The `target_altitude` argument sets the altitude the drone wants to fly
    so the voxmap will be centered in that altitude.

    The `voxel_size` argument sets the resolution of the voxel map.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    #alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    alt_max = 2 * target_altitude

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min))) // voxel_size
    east_size = int(np.ceil((east_max - east_min))) // voxel_size
    alt_size = int(alt_max) // voxel_size

    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        # fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True

        north1 = int(north - north_min - d_north - safety_distance) // voxel_size
        east1 = int(east - east_min - d_east - safety_distance) // voxel_size

        north2 = int(np.ceil((north - north_min + d_north + safety_distance) / voxel_size))
        east2 = int(np.ceil((east - east_min + d_east + safety_distance) / voxel_size))
        alt2 = int(np.ceil((alt + d_alt + safety_distance) / voxel_size))

        voxmap[north1:north2, east1:east2, 0:alt2] = True

    return voxmap, int(north_min), int(east_min)

### 3D A*
class Action3D(Enum):
    """
    An action is represented by a 4 element tuple.

    The first 3 values are the delta of the action relative
    to the current voxel position. The final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)

    # Diagonal motions
    N_WEST = (-1, -1, 0, np.sqrt(2))
    N_EAST = (-1, 1, 0, np.sqrt(2))
    S_WEST = (1, -1, 0, np.sqrt(2))
    S_EAST = (1, 1, 0, np.sqrt(2))

    # Up & down motions
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])

def valid_actions_3D(voxel, current_node):
    """
    Returns a list of valid actions given a voxel and current node.
    """
    valid_actions = list(Action3D)
    n, m, max_alt = voxel.shape[0] - 1, voxel.shape[1] - 1, voxel.shape[2] - 1
    x, y, z = current_node

    # check if the node is off the voxel or
    # it's an obstacle
    if z - 1 < 0 or voxel[x, y, z - 1] == 1:
        valid_actions.remove(Action3D.DOWN)
    if z + 1 > max_alt or voxel[x, y, z + 1] == 1:
        valid_actions.remove(Action3D.UP)

    if x - 1 < 0 or voxel[x - 1, y, z] == 1:
        valid_actions.remove(Action3D.NORTH)
        valid_actions.remove(Action3D.N_WEST)
        valid_actions.remove(Action3D.N_EAST)
    if x + 1 > n or voxel[x + 1, y, z] == 1:
        valid_actions.remove(Action3D.SOUTH)
        valid_actions.remove(Action3D.S_WEST)
        valid_actions.remove(Action3D.S_EAST)

    if y - 1 < 0 or voxel[x, y - 1, z] == 1:
        valid_actions.remove(Action3D.WEST)
        if Action3D.N_WEST in valid_actions:
            valid_actions.remove(Action3D.N_WEST)
        if Action3D.S_WEST in valid_actions:
            valid_actions.remove(Action3D.S_WEST)
    if y + 1 > m or voxel[x, y + 1, z] == 1:
        valid_actions.remove(Action3D.EAST)
        if Action3D.N_EAST in valid_actions:
            valid_actions.remove(Action3D.N_EAST)
        if Action3D.S_EAST in valid_actions:
            valid_actions.remove(Action3D.S_EAST)

    return valid_actions

def a_star_3D(voxel, h, start, goal):

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
            for action in valid_actions_3D(voxel, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1], current_node[2] + da[2])
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
