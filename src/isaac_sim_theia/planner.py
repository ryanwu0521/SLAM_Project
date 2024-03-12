import graph

class Planner(): #parent class
    def __init__(self) -> None:
        pass

    def generate_global_plan(self, fgraph:graph.TraversalGraph,start_node,end_node):
        #Need start and end position
        #Needs access to traversal graph
        #Needs to be able to calculate edge cost (on the fly is fine)
        #Eventually will need particle filter
        pass

    def generate_local_trajectory(self,curr_loc,goal_loc):
        #Need current location and final location
        #Need turn speed and 
        pass

#need child class for Djikstras and custom controller
class Djikstras(Planner):
    pass

class TDSP(Planner):
    pass