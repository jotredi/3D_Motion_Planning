from enum import Enum
from queue import PriorityQueue
from scipy.spatial import Voronoi
from bresenham import bresenham
import numpy as np
import networkx as nx
import numpy.linalg as LA

from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree


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

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    # Define a list to hold Voronoi points
    points = []

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

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # Create a voronoi graph based on location of obstacle centres
    graph = Voronoi(points)

    # Check each edge from graph.ridge_vertices for collision
    edges = []  # Empty list to contain valid edges
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        # Test each pair p1 and p2 for collision using Bresenham
        pts = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        in_collision = False
        for p in pts:
            # check if outside of the map
            if np.amin(p) < 0 or p[0] >= grid.shape[0] or p[1] >= grid.shape[1]:
                in_collision = True
                break
            # check if collides with obstacle
            if (grid[p[0], p[1]] == 1):
                in_collision = True
                break

        # If the edge does not hit an obstacle
        # add it to the list
        if not in_collision:
            # array to tuple for future graph
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)

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

        north2 = int(north - north_min + d_north + safety_distance) // voxel_size
        east2 = int(east - east_min + d_east + safety_distance) // voxel_size
        alt2 = int(alt + d_alt + safety_distance) // voxel_size

        voxmap[north1:north2, east1:east2, 0:alt2] = True

    return voxmap, int(north_min), int(east_min)

# Assume all actions cost the same.
class Action(Enum):
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

    # Diagonal motions
    N_WEST = (-1, -1, np.sqrt(2))
    N_EAST = (-1, 1, np.sqrt(2))
    S_WEST = (1, -1, np.sqrt(2))
    S_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

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


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
        valid_actions.remove(Action.N_WEST)
        valid_actions.remove(Action.N_EAST)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
        valid_actions.remove(Action.S_WEST)
        valid_actions.remove(Action.S_EAST)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
        if Action.N_WEST in valid_actions:
            valid_actions.remove(Action.N_WEST)
        if Action.S_WEST in valid_actions:
            valid_actions.remove(Action.S_WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        if Action.N_EAST in valid_actions:
            valid_actions.remove(Action.N_EAST)
        if Action.S_EAST in valid_actions:
            valid_actions.remove(Action.S_EAST)

    return valid_actions

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


def graph_a_star(graph, h, start, goal):

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
            for neighbor in list(graph.neighbors(current_node)):
                # get next node
                next_node = neighbor
                branch_cost = current_cost + graph[current_node][next_node]['weight']
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
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
    return LA.norm(np.array(position) - np.array(goal_position))


#=============== Path Pruning ===============#
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=10):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is not None:
        pruned_path = [p for p in path]
        i = 0
        while (i < len(pruned_path)-2):
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2])

            # check collinearity
            if collinearity_check(p1, p2, p3):
                del pruned_path[i+1]
            else:
                i += 1
    else:
        pruned_path = path

    return pruned_path


#============== Random Sampling ==============#
def extract_polygons(data):
    """
    Creates polygon objects from obstacles in data

    Returns:
        polygons: list of polygons with height
        poly_center: list of center of polygons
    """
    polygons = []
    poly_center = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Extract the 4 corners of the obstacle
        cor1 = (north-d_north, east-d_east)
        cor2 = (north-d_north, east+d_east)
        cor3 = (north+d_north, east+d_east)
        cor4 = (north+d_north, east-d_east)

        corners = [cor1, cor2, cor3, cor4]

        # Compute the height of the polygon
        height = alt + d_alt

        # Define polygons
        p = Polygon(corners)
        polygons.append((p, height))
        # Define polygon center
        poly_center.append((north, east))

    return polygons, poly_center

def collides(polygons, poly_tree, max_poly, point):
    """
    Checks if a point collides with any obstacle

    Args:
        polygons: list of obstacles represented as polygons
        poly_tree: KDTree containing centers of polygons
        max_poly: maximum polygon dimension
        point: point to check for collision

    Returns:
        True if the point collides with any obstacle
        and False if no collision is detected
    """
    p = Point(point)
    alt = point[2]

    # Find closest polygons to this point within 'max_poly' radius
    idxs = list(poly_tree.query_radius(np.array([point[0], point[1]]).reshape(1, -1), r=max_poly)[0])

    if len(idxs):
        for i in idxs:
            poly = polygons[i][0]
            height = polygons[i][1]

            if poly.contains(p) and alt <= height:
                return True
    return False

def Sampler(data, num_samples, z_max):
    """
    Sample random 3D points

    Args:
        data: obstacle data
        num_samples: number of points to be sampled
        z_maz: sample limit in the z axis

    Returns:
        to_keep: list of random 3D points free of collision
        polygons: list of obstacles represented as polygons
        poly_tree: KDTree containing centers of polygons
    """

    # Extract polygons
    polygons, poly_center = extract_polygons(data)
    # Define KDTree for centers of polygons
    poly_tree = KDTree(np.array(poly_center))

    # Define x, y & z ranges
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])

    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])

    zmin = 0
    zmax = z_max

    # Maximum polygon dimension in xy plane
    max_poly = 2 * np.max((data[:, 3], data[:, 4]))

    # Sample points
    xvals = np.random.uniform(xmin, xmax, num_samples)
    yvals = np.random.uniform(ymin, ymax, num_samples)
    zvals = np.random.uniform(zmin, zmax, num_samples)

    samples = list(zip(xvals, yvals, zvals))

    # Check points for collisions
    to_keep = []
    for point in samples:
        if not collides(polygons, poly_tree, max_poly, point):
            to_keep.append(point)

    return to_keep, polygons, poly_tree

#============ Probabilistic Roadmap ============#
def can_connect(p1, p2, polygons, poly_tree):
    """
    Checks if two nodes can be connected checking for collisions

    Args:
        p1, p2: points that represent the nodes
        polygons: list of obstacles represented as polygons
        poly_tree: KDTree containing centers of polygons

    Returns:
        True if connection is possible, False otherwise
    """
    # Cast as LineString()
    line = LineString([p1, p2])
    alt = np.min([p1[2], p2[2]])

    for p in polygons:
        poly = p[0]
        height = p[1]

        if poly.crosses(line) and alt <= height:
            return False
    return True

def create_graph(nodes, polygons, poly_tree):
    """
    Creates a graph from nodes testing for connectivity
    between each node and k of its nearest neighbors

    Args:
        nodes: list of points to be connected
        polygons: list of obstacles represented as polygons
        poly_tree: KDTree containing centers of polygons

    Returns:
        g: networkx graph
    """
    # Define network
    g = nx.Graph()
    # Define tree with nodes
    node_tree = KDTree(np.array(nodes))

    # Iterate through all nodes
    for node in nodes:
        # Nearest nodes
        idxs = node_tree.query([node], k=10, return_distance=False)[0]
        # Test for connectivity
        for i in idxs:
            if nodes[i] == node:
                continue

            if can_connect(node, nodes[i], polygons, poly_tree):
                # Distance between nodes
                dist = LA.norm(np.array(node) - np.array(nodes[i]))
                g.add_edge(node, nodes[i], weight=dist)
    return g

def prob_roadmap(data, num_samples, z_max):
    """
    Creates a probabilistic roadmap by
    randomly sampling the environment and returning
    a connected graph free of collisions

    Args:
        data: obstacle data
        num_samples: number of points to be sampled
        z_maz: sample limit in the z axis

    Returns:
        g: graph containing random points
    """

    # Sample points
    nodes, polygons, poly_tree = Sampler(data, num_samples, z_max)

    # Create a graph
    g = create_graph(nodes, polygons, poly_tree)

    return g
