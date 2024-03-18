from omni.isaac.core.prims import XFormPrim
from omni.isaac.debug_draw import _debug_draw

import omni.isaac.core.utils.stage as stage_utils

import agent as a

from cTheia import *

import scipy.stats as st

from queue import PriorityQueue
import math
import copy 
import numpy as np
import random
import warnings 

class Theia(a.Agent):
    #class to initialize a forklift instance
    #Needs a behavior tree (new class)
    #Needs to own the particle filter (new class)
    def __init__(self, fgraph:TraversalGraph, json_path:str, world) -> None:

        super().__init__(fgraph,json_path,world)
        self.tracked_objects = []
        for object in self.data["tracked_objects"]:
            self.tracked_objects.append(TrackedObject(self,object,self.graph,self.world))
        self.wait_time   = self.data["wait_time"]
        self.replan_flag = [False] * len(self.tracked_objects)
        #need TDSP Planner
        #take list of tracked objects and run update their individual particle filters

    def update_tracked_objects(self):
        self.tracked_objects.update()
    
    def collision_reset(self):
        pass

    def spin_once(self):
        for object in self.tracked_objects:
            object.spin_once()
        super().spin_once()

    def sync_world_pose(self):
        super().sync_world_pose()
        test_pose = self.prim.get_world_pose()
        test_orientation = Rotation.from_quat(test_pose[1])     

    def draw(self):
        pass
        # for object in self.tracked_objects:
        #     self.tracked_objects.draw()   

    def generate_global_plan(self):
            if self.current_node == None and self.current_edge == None:
                raise Exception("Current location unknown, shouldn't be possible")
            if self.current_node == None:
                raise Exception("Can only create global plan when at node location")
            if self.goal_node == None:
                warnings.warn("Tried to generate global plan with no set goal, generating a random goal")
                self.randomize_goal()
                
            self.tdsp()
            # self.dijkstras()

            if self.current_node != self.global_plan.pop(0):#verifying 
                raise Exception("Somehow the node the object is at and the start node for the plan is not the same")
            
            if self.global_plan[-1] != self.goal_node:
                raise Exception("planning algorithm resulted in different final location")
            
            if len(self.global_plan) != 0:
                self.set_next_node(self.global_plan.pop(0)) #Priming next node 

            self.replan_flag = [False] * len(self.tracked_objects)

    def update_position(self):
        if self.current_trajectory == None and (self.global_plan == None or len(self.global_plan) == 0) and self.goal_node == None and self.next_node == None:#Return catch for uninitialized case
            self.randomize_goal()
            return
        if self.current_trajectory == None and (self.global_plan == None or len(self.global_plan) == 0) and self.next_node == None: #Case where goal is set but plan hasn't been generated
            self.generate_global_plan()

        if self.current_trajectory == None: #Case where local trajectory hasn't been generated but is ready to be
            self.generate_local_trajectory()
        

        self.current_pose = self.current_trajectory.get_pose_at_time(self.current_time)

        # If I am turning and newly observe the object I replan
        for i in range(len(self.tracked_objects)):
            object = self.tracked_objects[i]
            if self.world.current_time < self.current_trajectory.turn_time and object.last_observed_time < self.current_trajectory.start_time:
                self.replan_flag[i] = True
            if self.replan_flag[i] == True and object.last_observed_time > self.current_trajectory.start_time:
                self.current_trajectory = None
                self.global_plan = None
                self.clear_next_node()
                return

        if self.world.current_time >= self.current_trajectory.turn_time:
            self.replan_flag = [False] * len(self.tracked_objects)

        if self.current_trajectory.is_finished(self.current_time):
            self.current_trajectory = None
            self.current_node = self.next_node
            self.clear_next_node()

            if len(self.global_plan) == 0:
                self.global_plan = None
                self.clear_goal()
            else: #replan at every new node
                self.global_plan = None
            return
        
    def tdsp(self):
        if self.current_node == None:
            raise Exception("Tried to plan without known starting location")
        if self.goal_node == None:
            raise Exception("Tried to plan before setting goal location")
        if self.current_node == self.goal_node:
            raise Exception("Goal and start node are the same")
        
        pq = PriorityQueue()
        best_cost = 0
        best_path = [self.current_node]
        pq.put(CompPath(best_cost,best_path,self.world.current_time))

        fnode = None

        while fnode != self.goal_node:
            if pq.empty():
                raise Exception("Failed to find path due to empty priority queue")
            temp = pq.get()
            best_cost = temp.cost 
            best_path = temp.path
            current_time = temp.time

            fnode = best_path[-1]

            start_pose = None
            if len(best_path) == 1:
                start_pose = Pose(position=fnode.get_position(),orientation=self.current_pose.get_orientation())
            else:
                start_pose = Pose(position=fnode.get_position())
                start_pose.set_heading_from_origin(best_path[-2].get_position())

            object_cost = 0
            fnode.visited = True
            for edge in fnode.edges:
                #skip visited nodes
                onode = edge.get_connected_node(fnode)
                if onode.visited:
                    continue

                #calculate cost to take edge
                #time it takes to travel distance
                cost_traj = Trajectory(start_pose,onode,edge,self.linear_speed,self.angular_speed,current_time,0)
                
                #TODO: Need to query edge for trajectory
                object_cost = self.calculate_cost(cost_traj)
                new_cost  = best_cost+(cost_traj.end_time-cost_traj.start_time)+object_cost

                #create path based on appending resulting node to best_path
                new_path = copy.copy(best_path)
                new_path.append(onode)
                #put cost path combo in pq
                pq.put(CompPath(new_cost,new_path,cost_traj.end_time))
            if self.wait_time > 0: #add wait edge for planning
                end_pose = start_pose
                cost_traj = Trajectory(start_pose,end_pose,self.linear_speed,self.angular_speed,current_time,wait_time=self.wait_time)
                #TODO: Query cost via trajectory
                new_cost  = best_cost+(cost_traj.end_time-cost_traj.start_time)+self.calculate_cost(cost_traj)
                new_path = copy.copy(best_path)
                new_path.append(onode)
                pq.put(CompPath(new_cost,new_path,cost_traj.end_time))
        
        self.global_plan = best_path
        for node in self.graph.nodes:
            node.visited = False

    def calculate_cost(self,trajectory):
        cost = 0
        for object in self.tracked_objects:
            cost += object.calculate_cost(trajectory)
        return cost

class TrackedObject():
    def __init__(self,host_agent,fdict,fgraph,world) -> None:
        self.graph              = fgraph
        self.world              = world
        self.host_agent         = host_agent
        self.prim_path          = fdict["prim_path"]
        self.observation_style  = fdict["observation_style"]
        self.dropout_time       = fdict["dropout_time"]
        self.avoidance_cost     = fdict["avoidance_cost"]
        
        self.particle_filter_dict    = fdict["particle_filter_dict"]


        self.prim               = XFormPrim(self.prim_path)
        self.particle_filter = None

        self.last_observed_time = 0

    def get_pose(self):
        pose = self.prim.get_world_pose()
        orientation_scalar_first = pose[1]
        orientation_scalar_last  = np.zeros(4)
        orientation_scalar_last[3]  = orientation_scalar_first[0]
        orientation_scalar_last[:3]  = orientation_scalar_first[1:]
        fpose = Pose(pose[0])
        fpose.set_heading_from_isaac_quat(pose[1])
        return fpose

    def dropout(self):
        #clear particle filter
        for edge in self.graph.edges:
            edge.draw_color = (1,0,0,.25)
        self.particle_filter = None
        
    def is_observed(self) -> bool:
        if self.observation_style == "FOV":
            return self.is_observed_FOV()
        elif self.observation_style == "ALL":
            return True
    
    def is_observed_FOV(self) -> bool:
        heading1 = self.host_agent.current_pose.get_heading_from_orientation()
        delta_pos = np.array(self.get_pose().get_position()) - np.array(self.host_agent.current_pose.get_position())
        heading2 = math.atan2(delta_pos[1],delta_pos[0])/math.pi*180
        delta_heading = heading2 - heading1

        while delta_heading > 180:
            delta_heading -= 360
        while delta_heading < -180:
            delta_heading += 360

        if abs(delta_heading) < 50: #TODO:Parameterize FOV
            return True

        return False

    def spin_once(self):
        observed = self.is_observed()
        if self.particle_filter == None:
            if observed:
                self.particle_filter = ParticleFilter(self.particle_filter_dict,self.graph,self.get_pose(),self.world.current_time)
            return
        
        self.particle_filter.trim_history(self.world.current_time-1)
        self.particle_filter.propogate_particles(self.world.current_time)
        
        if observed:
            self.particle_filter.update_particles(self.get_pose(),self.world.current_time)
            self.last_observed_time = self.world.current_time
        elif self.world.current_time - self.last_observed_time > self.dropout_time:
            self.dropout()
        else:
            self.particle_filter.clear_visible_particles(self.host_agent.current_pose,self.world.current_time)        

    def calculate_cost(self,trajectory:Trajectory):
        if self.particle_filter == None:
            return 0
        return (self.particle_filter.calculate_cost(trajectory))**2 * self.avoidance_cost
    
class ParticleFilter():
    def __init__(self,dict,fgraph:TraversalGraph,pose:Pose,time:float) -> None: 
        #need array of particles 

        self.graph = fgraph
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.draw_color = np.array([1, 1, 0, .1])
        self.draw_size  = 20
        self.spawn_rate         = dict["spawn_rate"]
        self.avg_speed          = dict["avg_speed"]
        self.dev_speed          = dict["dev_speed"]
        self.avg_rot            = dict["avg_rot"]
        self.dev_rot            = dict["dev_rot"]
        self.behavior           = dict["behavior"]
        self.num_particles      = dict["num_particles"]
        self.position_variance  = dict["position_variance"]
        self.angle_variance     = dict["angle_variance"]
        self.update_rate        = dict["update_rate"]

        self.edge = None
        self.last_update = 0

        self.particles = None
        self.initialize_particles(pose,time)
        
    def initialize_particles(self,pose,time):
        self.particles = [None] * self.num_particles
        edge = self.graph.get_closest_edge(pose.get_position())
        # edge.draw_color = (0, 1, 1, .25)
        next_node = None
        if edge.node1.x == pose.get_position()[0] and edge.node1.y == pose.get_position()[1]:
            next_node = edge.node1
        elif edge.node2.x == pose.get_position()[0] and edge.node2.y == pose.get_position()[1]:
            next_node = edge.node2
        else:
            heading = pose.get_heading_from_orientation()
            angle1 = math.atan2(edge.node1.y - pose.get_position()[1] , edge.node1.x - pose.get_position()[0])/math.pi*180
            angle2 = math.atan2(edge.node2.y - pose.get_position()[1] , edge.node2.x - pose.get_position()[0])/math.pi*180 
            del_1  = abs(angle1 - heading)
            if del_1 > 180:
                del_1 = 360 - del_1

            del_2  = abs(angle2 - heading)

            if del_2 > 180:
                del_2 = 360 - del_2

            p1 = del_2/(del_1+del_2)

            for i in range(self.num_particles):
                rand_float = random.random()
                if rand_float < p1:
                    next_node = edge.node1
                else:
                    next_node = edge.node2
                self.particles[i] = Particle(pose, edge, next_node, self.graph, self.rand_speed(), self.rand_rot(), self.behavior,time)
            return
        for i in range(self.num_particles):
            self.particles[i] = Particle(pose, edge,next_node, self.graph, self.rand_speed(), self.rand_rot(), self.behavior,time)
        return
    
    def clear_visible_particles(self,pose,time):
        if time - self.last_update > self.update_rate:
            self.last_update = time
        else:
            return
        
        likelihoods = np.ones(self.num_particles)*.5
        particle_seen = [False] * self.num_particles
        heading1 = pose.get_heading_from_orientation()
        for i in range(self.num_particles):
            ppose = self.particles[i].get_pose_at_time(time)
            dposition = np.array(ppose.get_position()) - np.array(pose.get_position())
            heading2 = math.atan2(dposition[1],dposition[0])/math.pi*180
            heading_to_particle = abs(heading2 - heading1)
            if heading_to_particle > 180:
                heading_to_particle -=360
            if heading_to_particle < -180:
                heading_to_particle += 360

            if abs(heading_to_particle) < 50:
                likelihoods[i] = .01
                particle_seen[i] = True


        if not (True in particle_seen):
            return
        likelihoods = likelihoods/np.linalg.norm(likelihoods)

        for i in range(self.num_particles):
            if not particle_seen[i]:
                continue
            p_sum = 0
            p = random.random()
            for j in range(self.num_particles):
                p_sum += likelihoods[j]
                if p_sum > p:
                    sample = self.particles[j]
                    self.particles[i] = Particle(sample.get_pose_at_time(time),sample.get_edge_at_time(time),sample.get_next_node_at_time(time),self.graph,self.rand_speed(),self.rand_rot(),self.behavior,time)
                    break

    def update_particles(self, pose, time):
        
        if time - self.last_update > self.update_rate:
            self.last_update = time
        else:
            return
        likelihoods = np.zeros(self.num_particles)
        new_particles = []
        if self.edge == None or self.edge.get_distance_from(pose.get_position()) > .02:
            self.edge = self.graph.get_closest_edge(pose.get_position())

        next_node = None
        ratio = 1-self.spawn_rate
        if self.edge.node1.x == pose.get_position()[0] and self.edge.node1.y == pose.get_position()[1]:
            p1 = 1 
        elif self.edge.node2.x == pose.get_position()[0] and self.edge.node2.y == pose.get_position()[1]:
            p1 = 0
        else:
            heading = pose.get_heading_from_orientation()
            angle1 = math.atan2(self.edge.node1.y - pose.get_position()[1] , self.edge.node1.x - pose.get_position()[0])/math.pi*180
            angle2 = math.atan2(self.edge.node2.y - pose.get_position()[1] , self.edge.node2.x - pose.get_position()[0])/math.pi*180 
            del_1  = abs(angle1 - heading)
            if del_1 > 180:
                del_1 = 360 - del_1

            del_2  = abs(angle2 - heading)

            if del_2 > 180:
                del_2 = 360 - del_2

            p1 = del_2/(del_1+del_2)

        for i in range(self.num_particles):
            ppose = self.particles[i].get_pose_at_time(time)

            z_pos   = np.linalg.norm(np.array(ppose.get_position()) - np.array(pose.get_position()))/self.position_variance
            d_rot   = abs(ppose.get_heading_from_orientation() - pose.get_heading_from_orientation())
            if d_rot > 180:
                d_rot = 360 - d_rot
            z_rot   = d_rot/self.angle_variance

            likelihoods[i] = st.norm.cdf(z_pos)*st.norm.cdf(z_rot)

        likelihoods = likelihoods/np.linalg.norm(likelihoods)*ratio

        for i in range(self.num_particles):
            p_sum = 0
            p = random.random()
            for j in range(self.num_particles):
                p_sum += likelihoods[j]
                if p > ratio:
                    rand_float = random.random()
                    if rand_float < p1:
                        next_node = self.edge.node1
                    else:
                        next_node = self.edge.node2
                    new_particles.append(Particle(pose,self.edge,next_node,self.graph,self.rand_speed(),self.rand_rot(),self.behavior,time))
                    break
                elif p_sum > p:
                    sample = self.particles[j]
                    new_particles.append(Particle(sample.get_pose_at_time(time), sample.get_edge_at_time(time), sample.get_next_node_at_time(time), self.graph,self.rand_speed(),self.rand_rot(),self.behavior,time))
                    break

        self.particles = new_particles

    def propogate_particles(self,prop_time):
        for particle in self.particles:
            particle.propogate(prop_time)

    def draw_particles(self,time,color=None):
        if color == None:
            color = self.draw_color
        
        self.propogate_particles(time)
        for particle in self.particles:
            self.draw.draw_points([particle.get_pose_at_time(time).get_position()],[color],[self.draw_size])

    def trim_history(self,trim_time):
        for particle in self.particles:
            particle.trim_history(trim_time)
    
    def rand_speed(self):
        return random.normalvariate(self.avg_speed,self.dev_speed)

    def rand_rot(self):
        return random.normalvariate(self.avg_rot,self.dev_rot)
    
    def calculate_cost(self,trajectory):
        cost = 0
        cost_inc = 1.0/self.num_particles
        for particle in self.particles:
            if particle.collision_check(trajectory):
                cost += cost_inc
        return cost