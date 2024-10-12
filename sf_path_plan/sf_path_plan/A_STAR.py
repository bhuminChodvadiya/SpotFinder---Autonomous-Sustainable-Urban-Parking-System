import math
from queue import PriorityQueue
import io

class Node:
    '''A class representing a node in a graph with attributes for A* pathfinding.

        Attributes:
            node_id (int): The unique identifier for the node.
            x (float): The x-coordinate of the node.
            y (float): The y-coordinate of the node.
            g (float): Cost from the start node to this node.
            h (float): Heuristic estimate from this node to the end node.
            f (float): Total cost (g + h).
            parent (Node): The parent node in the path.
    '''
    def __init__(self, node_id, x, y):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f # Compare nodes based on f-cost for priority queue

    def __eq__(self, other):
        return self.node_id == other.node_id # Check node equality based on node ID


def parse_map_data(file_path, traffic_data):
    ''' 
    Parse map data from a file to create nodes and edges for the graph.
    '''
    nodes = {}
    edges = []

    edges_id_dict = {
        "JI": (21, 22),
        "IH": (19, 20),
        "HG": (16, 17, 18),
        "JC": (1, 2, 3),
        "ID": (35, 36, 37, 45, 46, 47),
        "HE": (32, 33, 34, 42, 43, 44),
        "GF": (13, 14, 15),
        "CD": (4, 5),
        "DE": (6, 7),
        "EF": (8, 9, 10, 11, 12),
        "CA": (24, 25, 26),
        "DB": (28, 29, 30, 31),
        "AB": (27, 28)
        }
    with open(file_path, 'r') as f:
        data = f.read()

    file = io.StringIO(data)  # Create a file-like object from the string data
    lines = file.readlines()
    for line in lines:
        if line.startswith('n'): # Check if the line defines a node
            node = line.split()
            node_id = int(node[3])  # node ID
            x = float(node[1])  # The node's x-coordinate
            y = float(node[2])  # The node's y-coordinate
            nodes[node_id] = Node(node_id, x, y)
        elif line.startswith('e'):   # Check if the line defines an edge
            edge = line.split()
            edge_density_map = {}
            # Populate the edge_density_map
            for area, edge_ids in edges_id_dict.items():
                density = traffic_data[area]  # Direct access instead of using .get()
                for edge_id in edge_ids:
                    edge_density_map[edge_id] = {'area': area, 'density': density}

            for edge_id, traffic_info in edge_density_map.items():
                if  str(edge_id) == edge[3]:
                    print(edge[3])
                    print(edge_id)
                    traffic_cost = traffic_info['density'] * 5000
                    print(int(edge[1]))
                    print(int(edge[2]))
                    print(float(edge[4]))
                    print(traffic_cost)
                    edges.append((int(edge[1]), int(edge[2]), float(edge[4]), float(traffic_cost))) # Append (start_node_id, end node ID, cost,)
     
    return nodes, edges

def calculate_distance(point1, point2):
    '''Calculate the Euclidean distance between two points.'''
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) # Calculate and return the Euclidean distance

def get_closest_node_id(nodes, position):
    '''Find the closest node to a given position.'''
    if not nodes:
        raise ValueError("No nodes available to find the closest node.")
    closest_node = min(nodes.values(), key=lambda node: calculate_distance((node.x, node.y), position))
    return closest_node.node_id # Return the ID of the closest node

def h(node1, node2):
    '''Heuristic function for A* pathfinding.'''
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def astar(nodes, edges, start_position, end_position):
    '''
    Perform A* pathfinding algorithm to find the shortest path between two positions.
    '''
    # Get the closest node ID to the start position and end position
    start_id = get_closest_node_id(nodes, start_position)
    end_id = get_closest_node_id(nodes, end_position)

    # Initialize the priority queue for the open list and the closed list as a set
    open_list = PriorityQueue()
    closed_list = set()

    # Get the start node and end node
    start_node = nodes[start_id]
    end_node = nodes[end_id]

    start_node.g = 0                        # Set the g-cost of the start node to 0
    start_node.f = h(start_node, end_node) # Set the f-cost of the start node using the heuristic

    open_list.put(start_node)          # Add the start node to the open list
    while not open_list.empty():
        current_node = open_list.get() # Get the node with the lowest f-cost

        if current_node.node_id in closed_list:  # Skip the node if it's already in the closed list
            continue
        closed_list.add(current_node.node_id)  # Add the current node to the closed list

        if current_node.node_id == end_node.node_id:   # Check if the end node has been reached
            path = []
            while current_node:  # Trace back the path from end node to start node
                path.append(current_node.node_id)
                current_node = current_node.parent
            return path[::-1]# Return path

        for start, end, cost, traffic_cost in edges:  # Iterate over each edge
            if current_node.node_id == start: # Check if the current node is the start of the edge
                neighbor = nodes[end]  # Get the neighbor node
            else:
                continue

            if neighbor.node_id in closed_list:  # Skip the neighbor if it's already in the closed list
                continue

            temp_g = current_node.g + cost + traffic_cost # Calculate the tentative g-cost for the neighbor
            if temp_g < neighbor.g:  # Check if the tentative g-cost is less than the current g-cost
                neighbor.parent = current_node  # Set the current node as the parent of the neighbor
                neighbor.g = temp_g             # Update the g-cost of the neighbor
                neighbor.h = h(neighbor, end_node) # Update the h-cost of the neighbor
                neighbor.f = neighbor.g + neighbor.h # Update the f-cost of the neighbor
                open_list.put(neighbor)  # Add the neighbor to the open list
    return None # Return None if no path is found