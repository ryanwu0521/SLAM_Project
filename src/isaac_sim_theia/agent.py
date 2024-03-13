from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.stage as stage_utils
import cTheia as g
import json
import random
import numpy as np
from queue import PriorityQueue
import copy
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import math 
import warnings

class Agent():#Note, one day this probably should just be an inherited class from the omni.isaac.core robot class but I didn't do that because I'd have to learn more about that class

    #Needs a behavior tree (new class)
    #Needs to own the particle filter (new class)
    def __init__(self, fgraph:g.TraversalGraph, json_path:str, world):
        file = open(json_path)
        self.data = json.load(file)
        self.world = world
        self.current_node = None
        self.current_edge = None
        self.goal_node    = None
        self.next_node    = None
        self.global_plan  = None
        self.current_trajectory = None

        self.graph      = fgraph
        self.goal_color = self.data["goal_color"]
        self.next_color = self.data["next_color"]

        self.name       = self.data["name"]
        self.usd_path   = self.data["usd_path"]
        self.prim_path  = self.data["prim_path"]
        self.articulation_root = self.data["articulation_root"]
        self.linear_speed = self.data["linear_speed"]
        self.angular_speed = self.data["angular_speed"]
        self.height_offset = self.data["height_offset"] #TODO: replace with param

        self.current_pose = Pose(np.array([0,0,0]),R.from_euler('z',0,degrees=True))
        self.current_time = self.world.current_time

        stage_utils.add_reference_to_stage(self.usd_path,self.prim_path)

        self.prim       = XFormPrim(self.articulation_root, name = self.name, position = self.current_pose.position, orientation = self.current_pose.get_quat_scalar_first())
            
    def sync_to_world_time(self):
        self.current_time = self.world.current_time

    def set_position_to_node(self, node, orientation):
        self.current_node = node
        self.current_pose.position = [self.current_node.x,self.current_node.y,0]
        self.current_pose.set_heading_from_angle(orientation)
        self.sync_world_pose()


    def randomize_position(self):
        rand_node = random.choice(self.graph.nodes)
        self.current_node = rand_node
        self.current_pose.position = (rand_node.x,rand_node.y,0)
        self.current_pose.randomize_orientation()
        self.sync_world_pose()

    def generate_local_trajectory(self):
        if self.next_node == None and self.global_plan == None:
            raise Exception("Tried to generate trajectory with insufficient information")
        
        if self.next_node == None: #TODO: Decide if necessary
            self.next_node = self.global_plan.pop(0)
        
        fend_position   = np.copy(self.next_node.get_position())
        fend_position[2] = self.height_offset

        fend_pose       = Pose(position=fend_position)
        fend_pose.set_heading_from_origin(self.current_pose.position)
        
        self.current_trajectory = Trajectory(start_pose=self.current_pose,end_pose=fend_pose,linear_speed=self.linear_speed,angular_speed=self.angular_speed,start_time=self.current_time)
        

    def generate_global_plan(self):
        if self.current_node == None and self.current_edge == None:
            raise Exception("Current location unknown, shouldn't be possible")
        if self.current_node == None:
            raise Exception("Can only create global plan when at node location")
        if self.goal_node == None:
            warnings.warn("Tried to generate global plan with no set goal, generating a random goal")
            self.randomize_goal()
        #TODO:Handling for goal node is current node
            
        self.dijkstras()

        if self.current_node != self.global_plan.pop(0):#verifying 
            raise Exception("Somehow the node the object is at and the start node for the plan is not the same")
        
        if self.global_plan[-1] != self.goal_node:
            raise Exception("planning algorithm resulted in different final location")
        
        if len(self.global_plan) != 0:
            self.set_next_node(self.global_plan.pop(0)) #Priming next node  

    def set_goal(self,node):
        #Set goal to given node
        self.clear_goal()

        self.goal_node = node
        # self.goal_node.draw_color = self.goal_color

    def set_next_node(self,node):
        self.clear_next_node()

        self.next_node = node
        # self.next_node.draw_color = self.next_color

    def clear_goal(self):
        if self.goal_node != None:
            self.goal_node.reset_color()
        
        self.goal_node = None

    def clear_next_node(self):
        if self.next_node != None:
            self.next_node.reset_color()
        
        self.next_node = None


    def randomize_goal(self):

        fgoal_node = random.choice(self.graph.nodes)
        if fgoal_node == self.current_node:
            self.randomize_goal()
        else:
            self.set_goal(fgoal_node)
    
    def sync_world_pose(self):
        self.prim.set_world_pose(position = self.current_pose.position, orientation=self.current_pose.get_quat_scalar_first())

    def update_position(self):
        if self.current_trajectory == None and (self.global_plan == None or len(self.global_plan) == 0) and self.goal_node == None and self.next_node == None:#Return catch for uninitialized case
            self.randomize_goal()
            return
        if self.current_trajectory == None and (self.global_plan == None or len(self.global_plan) == 0) and self.next_node == None: #Case where goal is set but plan hasn't been generated
            self.generate_global_plan()

        if self.current_trajectory == None and self.next_node != None: #Case where local trajectory hasn't been generated but is ready to be
            self.generate_local_trajectory()
        
        if self.current_trajectory == None:
            raise Exception("This should never happen")

        self.current_pose = self.current_trajectory.get_pose_at_time(self.current_time)

        if self.current_trajectory.is_finished(self.current_time):
            self.current_trajectory = None
            self.current_node = self.next_node
            self.clear_next_node()

            if len(self.global_plan) == 0:
                self.global_plan == None
                
                self.clear_goal()
            else:
                self.set_next_node(self.global_plan.pop(0))


    def spin_once(self):
        #Holds all the necessary time step actions
        self.sync_to_world_time()
        self.update_position()
        self.sync_world_pose()

    def dijkstras(self):
        if self.current_node == None:
            raise Exception("Tried to plan without known starting location")
        if self.goal_node == None:
            raise Exception("Tried to plan before setting goal location")
        if self.current_node == self.goal_node:
            raise Exception("Goal and start node are the same")
        
        pq = PriorityQueue()
        best_cost = 0
        best_path = [self.current_node]
        pq.put(CompPath(best_cost,best_path))

        fnode = None
        while fnode != self.goal_node:
            if pq.empty():
                raise Exception("Failed to find path due to empty priority queue")
            temp = pq.get()
            best_cost = temp.cost 
            best_path = temp.path

            fnode = best_path[-1]
            start_pose = None
            if len(best_path) == 1:
                start_pose = Pose(position=fnode.get_position(),orientation=self.current_pose.orientation)
            else:
                start_pose = Pose(position=fnode.get_position())
                start_pose.set_heading_from_origin(best_path[-2].get_position())


            fnode.visited = True

            for edge in fnode.edges:
                #skip visited nodes
                onode = edge.get_connected_node(fnode)
                if onode.visited:
                    continue

                #calculate cost to take edge
                #time it takes to travel distance
                end_pose  = Pose(position=onode.get_position())
                end_pose.set_heading_from_origin(start_pose.position)
                cost_traj = Trajectory(start_pose,end_pose,self.linear_speed,self.angular_speed,0)
                
                new_cost  = best_cost+cost_traj.end_time

                #create path based on appending resulting node to best_path
                new_path = copy.copy(best_path)
                new_path.append(onode)
                #put cost path combo in pq
                pq.put(CompPath(new_cost,new_path))

        self.global_plan = best_path
        for node in self.graph.nodes:
            node.visited = False
           
class Pose():
    def __init__(self,position=np.array([0,0,0]),orientation=R.identity()) -> None:
        self.position    = np.array(position)
        self.orientation = orientation

    def __eq__(self, __value: object) -> bool:
        return (self.position[0] == object.position[0] and self.position[1] == object.position[1] and self.get_heading_from_orientation() == object.get_heading_from_orientation())

    def set_heading_from_isaac_quat(self,quat_s_first):
        quat_s_last = np.zeros(4)
        quat_s_last[3] = quat_s_first[0]
        quat_s_last[:3] = quat_s_first[1:]

        self.orientation = R.from_quat(quat_s_last)

    def get_quat_scalar_first(self):
        quat_s_last = self.get_quat_scalar_last()
        quat_s_first = np.zeros(4)
        quat_s_first[0] = quat_s_last[3]
        quat_s_first[1:] = quat_s_last[:3]
        return quat_s_first

    def get_quat_scalar_last(self):
        return self.orientation.as_quat()

    def set_heading_from_angle(self,angle,degrees=True):
        self.orientation = R.from_euler('z',angle,degrees=degrees)

    def set_heading_to_destination(self,dest):
        delta_pos = dest-self.position
        self.set_heading_from_angle(math.atan2(delta_pos[1],delta_pos[0]),degrees=False) 

    def set_heading_from_origin(self,origin):
        delta_pos = self.position-np.array(origin)
        self.set_heading_from_angle(math.atan2(delta_pos[1],delta_pos[0]),degrees=False)

    def randomize_orientation(self):
        self.orientation = R.from_euler('z',random.randint(0,360),degrees=True)

    def get_heading_from_orientation(self):
        return self.orientation.as_euler('zxy',degrees=True)[0]

class Trajectory():
    def __init__(self,start_pose,end_pose,linear_speed,angular_speed,start_time,wait_time=0) -> None: #TODO:possibly create wait time?
        self.start_pose     = start_pose
        self.end_pose       = end_pose
        self.linear_speed   = linear_speed
        self.angular_speed  = angular_speed
        self.start_time     = start_time
        self.wait_time      = 0
        #TODO: Add wait time support
        self._calculate_times()
        self.slerp          = None
    
    def _calculate_times(self):
        delta_angle = self.end_pose.get_heading_from_orientation() - self.start_pose.get_heading_from_orientation()
        while delta_angle < -180:
            delta_angle += 360
        while delta_angle > 180:
            delta_angle -= 360
        
        self.turn_time  = self.start_time + abs(delta_angle)/self.angular_speed + self.wait_time
        self.end_time = self.turn_time + np.linalg.norm(self.end_pose.position-self.start_pose.position)/self.linear_speed
    
    def _setup_slerp(self):
        key_rots    = R.concatenate([self.start_pose.orientation,self.start_pose.orientation,self.end_pose.orientation,self.end_pose.orientation])
        key_times   = [self.start_time,self.start_time+self.wait_time+.001,self.turn_time+.002,self.end_time+.003] #hacky bump off
        self.slerp  = Slerp(key_times,key_rots)

    def get_pose_at_time(self,time):
        if self.slerp == None:
            self._setup_slerp()

        if time <= self.start_time:
            return self.start_pose
        if time >= self.end_time:
            return self.end_pose
        
        if time < self.turn_time:
            return Pose(position=self.start_pose.position,orientation=self.slerp(time))
        
        if self.end_time <= self.turn_time and time >= self.turn_time:
            fposition = self.end_pose.position
        else:
            fposition = (time-self.turn_time)/(self.end_time-self.turn_time)*(self.end_pose.position-self.start_pose.position) + self.start_pose.position

        return Pose(position=fposition,orientation=self.slerp(time))

    def is_finished(self,time):
        return time > self.end_time

class CompPath():
    def __init__(self, cost, path, time=None) -> None:
        self.cost   = cost
        self.path   = copy.copy(path)
        self.time   = time
        if self.time == None:
            self.time   = cost

    def __eq__(self, other):
        return self.cost == other.cost
    
    def __ne__(self, other):
        return self.cost != other.cost

    def __lt__(self, other):
        return self.cost < other.cost
    
    def __le__(self, other):
        return self.cost <= other.cost
    
    def __gt__(self, other):
        return self.cost > other.cost
    
    def __ge__(self, other):
        return self.cost >= other.cost   
