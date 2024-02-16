from omni.isaac.debug_draw import _debug_draw
import json 
import random
import math
import numpy as np
from omni.isaac.debug_draw import _debug_draw

class TraversalGraph():
    def __init__(self, file_string) -> None:
        #Needs array of nodes 
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.nodes = []
        self.edges = []
        self.draw_height = .5
        self.load_file(file_string)

    def load_file(self, file_string):
        #parse through file structure to do connectivity
        print('loading graph from file ' + file_string)
        f = open(file_string)
        data = json.load(f)

        #need all nodes to exist to create referential edges
        for node in data['nodes']:
            self.nodes.append(TraversalNode(node))
        self.validate_nodes()

        for edge in data['edges']:
            #create all edges
            self.add_edge(edge)

        print('Traversal graph loaded successfully')
    
    def add_edge(self,edge):
        node1_key = edge['node1']
        node2_key = edge['node2']

        if node1_key == node2_key:
            raise Exception("Self referrential edge in json file")
        elif node1_key >= len(self.nodes):
            raise Exception("Invalid key for node 1")
        elif node2_key >= len(self.nodes):
            raise Exception("Invalid key for node 2")
        
        fnode1 = self.nodes[node1_key]
        fnode2 = self.nodes[node2_key]
              
        #create edge object and add to graph list
        self.edges.append(TraversalEdge(fnode1,fnode2))

    def validate_nodes(self):
        for i in range(len(self.nodes)):
            if(i!=self.nodes[i].key):
                raise Exception("Node key mismatch, check input .json file structure")

    def draw_graph(self): #Delegate to individual edges and nodes
        #TODO: Consider compiling draw list first to limit calls
        for node in self.nodes:
            self.draw.draw_points([node.get_position()],[node.draw_color],[node.draw_size])
        for edge in self.edges:
            self.draw.draw_lines([edge.node1.get_position()],[edge.node2.get_position()],[edge.draw_color],[edge.draw_size])

    def get_closest_edge(self,position):
        closest = math.inf
        cedge = None

        for edge in self.edges:
            d2e = edge.get_distance_from(position)
            if d2e == 0:
                return edge
            if d2e < closest:
                closest = d2e
                cedge = edge
        return cedge

class TraversalEdge():
    def __init__(self, node1, node2) -> None:
        self.node1 = node1
        self.node2 = node2

        self.default_color =  (1,0,0,.25)
        self.draw_color     = (1,0,0,.25)
        self.draw_size      = 5

        self.base_cost  = 0
        self.cost_table = []

        self.node1.add_edge(self)
        self.node2.add_edge(self)

        self.dist = math.sqrt((self.node1.x-self.node2.x)^2+(self.node1.x-self.node2.x)^2)
        
    def get_connected_node(self, node):
        if node.key == self.node1.key:
            return self.node2
        if node.key == self.node2.key:
            return self.node1
        raise Exception("Node not part of edge")
    
    def get_distance_from(self,position):
        #distance to line segment
        
        A = np.zeros(2)
        A[0] = self.node1.x
        A[1] = self.node1.y

        B = np.zeros(2)
        B[0] = self.node2.x
        B[1] = self.node2.y

        E  = position[:2]

        AB = B-A
        AE = E-A
        BE = E-B

        AB_BE = np.dot(AB,BE)
        AB_AE = np.dot(AB,AE)

        reqAns = 0
        
        if (AB_BE>0):
            reqAns = np.linalg.norm(E-B)
        elif(AB_AE<0):
            reqAns = np.linalg.norm(E-A)
        else:
            reqAns = abs(AB[0] * AE[1] - AB[1] * AE[0])/np.linalg.norm(AB)
        return reqAns

    def reset_color(self):
        self.draw_color = self.default_color


class TraversalNode():
    #Needs location (x,y)
    def __init__(self,node,draw_height=.5) -> None:
        self.edges = []
        self.x     = node['x']
        self.y     = node['y']
        self.key   = node['key']

        self.draw_height = draw_height
        self.default_color = (1,1,1,.5)
        self.draw_color    = self.default_color
        self.draw_size     = 20

        self.visited = False

    def get_position(self):
        return np.array([self.x,self.y,self.draw_height])
    
    def reset_color(self):
        self.draw_color = self.default_color

    def add_edge(self,new_edge):
        #Exception handling for failure conditions from json error
        if new_edge.node1 == self:
            for edge in self.edges:
                if new_edge.node2 == edge.node1 or new_edge.node2 == edge.node2:
                    raise Exception("Attempted to add duplicate edge")
        elif new_edge.node2 == self:
            for edge in self.edges:
                if new_edge.node1 == edge.node1 or new_edge.node1 == edge.node2:
                    raise Exception("Attempted to add duplicate edge")
        else: #If node is not part of edge
            raise Exception("Attempted to add edge to non-member node")

        self.edges.append(new_edge)
        
