import numpy as np 
from task import distance as dist

class Node(object):
    def __init__(self, coordinates):
        assert len(coordinates) == 3
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.box_id = coordinates[3]
        self.coords = tuple(coordinates[0], coordinates[1], coordinates[2])
        self.pos = np.array([self.x, self.y, self.z])

        if self.box_id > 0:
            self.associated_pheromone = 50
            self.time_decay = .8
            self.spatial_decay = .8
        else:
            self.associated_pheromone, self.time_decay, self.spatial_decay = 0, 0, 0

        self.neighbors = [] # list of Nodes
        self.distances_to_neighbors = {}
    
    def add_neighbors(self, neighbors):
        self.neighbors.extend(neighbors)

    def calculate_distances_to_neighbors(self):
        for n in self.neighbors:
            self.distances_to_neighbors[n] = dist(n.pos, self.pos)


class Box(object):
    def __init__(self, height, candidate_nodes, box_id, current_node):
        # Height is height of box
        # candidate_nodes is list of nodes represnting possible places it can be placed
        self.box_id = box_id
        self.height = height
        self.candidates = candidate_nodes
        self.current_node = current_node
        self.distances_to_candidates = {}  # map candidates to distance based on the map
        self.associated_pheromone = 50
        self.time_decay = .8
        self.spatial_decay = .8

class Robot(object):
    def __init__(self, current_node, robot_id):
        self.robot_id = robot_id
        self.current_node = current_node

class Graph(object):
    def __init__(self):
        self.nodes = []
        self.boxes = []
        self.robots = []
    
    def get_vrep_input_and_convert_to_nodes(self, node_data_msg, inclusions_msg, exclusions_msg, box_msg, robot_msg):
        """
        node_data_msg: list of tuples corresponding to: (node coordinates, box_id...0 if no box allowed)
        inclusions_msg = list of tuples correpsonding to (node index, node index)
        exclusions_msg = list of tuples corresponding to (node index, index)
        box_id_msg = list of box info tuples (index of current node, box_id, height)
        robot_msg: list of tuples (index of current node, robot_id)
        """
        nodes = node_data_msg.data  # List of tuplels. Tuples correspond to node coordinates and box_id (0 if no box)
        inclusions = inclusions_msg.data 
        inclusions = [(nodes[index1][0], nodes[index2][0]) for index1, index2 in inclusions]
        exclusions = exclusions_msg.data
        exclusions = [(nodes[index1][0], nodes[index2][0]) for index1, index2 in exclusions]
        box_data = box_msg.data 
        robot_data = robot_msg.data

        self.insert_nodes(nodes, 2, inclusions, exclusions)
        for index_of_current_node, box_id, height in box_data:
            current_node = self.nodes[index_of_current_node]
            candidate_nodes = [node for node in self.nodes if node.box_id == box_id]
            new_box = Box(height, candidate_nodes, box_id, current_node)
            self.boxes.append(new_box)
        
        for robot_curr_node, robot_id in robot_data:
            self.robots.append(Robot(self.nodes[robot_curr_node], robot_id))

    def insert_chain(self, chain):
        """
        Takes a chain (list) of coordinates to process as a path
        """
        for i, coord in enumerate(chain):
            curr = Node(coord)
            if i < len(chain) - 1:
                curr.neighbors.append(chain[i+1])
            self.nodes.append(Node(coord))

    def insert_nodes(self, coords, radius, inclusions, exclusions):
        """
        Takes a list of nodes (4-d tuples) to add into the graph. Radius represents closeness to add an edge. Exclusions is list of tuples representing exclusions from edges
        """
        for coord in coords:
            curr = Node(coord)
            self.nodes.append(curr)
        for node in self.nodes:
            neighbors = self.get_neighbors(node, radius, inclusions, exclusions)
            node.add_neighbors(neighbors)
            node.calculate_distances_to_neighbors()

    def get_neighbors(self, node, radius, inclusions, exclusions):
        neighbors = []
        for node_other in self.nodes:
            if (node.coords, node_other.coords) in exclusions:
                continue
            elif dist(node, node_other) < radius and node_other != node:
                neighbors.append(node_other)
            elif (node.coords, node_other.coords) in inclusions:
                neighbors.append(node_other)
        return neighbors

    def get_next_relative_coords(self, relative_pos, global_pos, node1, node2):
        """
        Given the robot perceives node1 to be at position "relative" to it and robot is at global_pos, find the position of node2 relative to the robot
        """
        one_to_two = node2.pos - node1.pos
        rob_to_one = node1.pos - global_pos 
        
        rob_to_two_global = rob_to_one + one_to_two 

        rotation = np.linalg.solve(rob_to_one, relative_pos).T
        relative_coords = np.matmul(rotation, one_to_two)
        return relative_coords

    def update_pheromones(self):
        return

class StigmergicAgent(object):
    def __init__(self):
        self.value = 0
        self.target_box_index = None
        self.target_node_index = None 
        self.current_node_position_index = None 
        self.mode = 'FIND_BOX' # 'TOWARDS_HOLE', 'ROAM'

    def calculate_value(self):
        return 0
    
    def choose_target(self):
        return 
    
class StigmergicWrapper(object):
    def __init__(self):
        return