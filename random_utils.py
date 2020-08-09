import numpy as np
import networkx as nx
import numpy.linalg as LA

from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree

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

    # Find closest polygons to this line
    idxs = poly_tree.query([(p1[0],p1[1])], k=5, return_distance=False)[0]
    idxs2 = poly_tree.query([(p2[0],p2[1])], k=5, return_distance=False)[0]

    for i in np.concatenate((idxs, idxs2), axis=None):
        poly = polygons[i][0]
        height = polygons[i][1]

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
