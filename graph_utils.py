from enum import Enum
from queue import PriorityQueue
from scipy.spatial import Voronoi
from bresenham import bresenham
import numpy as np

import networkx as nx

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


### A* for graphs
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
