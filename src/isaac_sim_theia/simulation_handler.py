import omni
from omni.isaac.core.utils.nucleus import is_file
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import is_stage_loading
# import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core import World
from omni.isaac.debug_draw import _debug_draw
from omni.isaac.core import PhysicsContext

import numpy as np
import carb 
import sys
import random

import cTheia as c 
import agent
import theia 
import time 

class QualSimulationHandler:

    def __init__(self, kit) -> None:
        
        #TODO: Add in functionality to choose world
        self.kit = kit
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.graph = c.TraversalGraph('src/isaac_sim_theia/graphs/test_world_1_graph.json')
        self.load_stage()

        self.world = World(physics_dt=.5,rendering_dt=.5)
        self.physx = self.world.get_physics_context()
        self.physx.set_gravity(0.0)
        self.world.initialize_physics()

        omni.timeline.get_timeline_interface().play()

        self.forklift_1   = agent.Agent(self.graph,"src/isaac_sim_theia/config/forklift.json",self.world) 
        # self.forklift_1.randomize_position()
        # self.forklift_1.randomize_goal()
        self.forklift_1.set_position_to_node(self.graph.nodes[0],0)
        self.forklift_1.set_goal(self.graph.nodes[48])
        # self.forklift_1.randomize_position()

        self.theia_1  = theia.Theia(self.graph,"src/isaac_sim_theia/config/theia_robot.json",self.world)
        self.theia_1.set_position_to_node(self.graph.nodes[5],180)
        self.theia_1.set_goal(self.graph.nodes[53])
        # self.theia_1.randomize_position()

        # while self.theia_1.current_node == self.forklift_1.current_node: #making sure they aren't overlapping to start
        #     self.theia_1.randomize_position()

        # self.theia_1.randomize_goal()
        # while self.theia_1.goal_node == self.forklift_1.goal_node: #making sure that they aren't routing to the exact same node
        #     self.theia_1.randomize_goal()

        self.kit.update()
        self.kit.update()
        self.kit.update()


    def draw_particles(self,time,color=np.array([1, 1, 0, .2])):
        
        for object in  self.theia_1.tracked_objects:
            object.particle_filter.propogate_particles(time)

            for particle in object.particle_filter.particles:
                self.draw.draw_points([particle.get_pose_at_time(time).get_position()],[color],[20])

    def draw_cool_filter(self,t):
        
        if self.theia_1.tracked_objects[0].particle_filter == None:
            return
        for i in range(t):
            self.draw_particles(self.world.current_time + i,(1,(i)/t,0,.1))

    def draw_update(self):
        #TODO: Consider compiling draw list first to limit calls
        self.draw_graph()
        # if self.theia_1.tracked_objects[0].particle_filter != None:
        #     self.theia_1.tracked_objects[0].particle_filter.draw_particles(self.world.current_time + 8)
        self.draw_path()
        self.draw_goal()
        self.draw_cool_filter(10)
        # self.draw_simple_robots()
        pass
    
    def draw_graph(self):
        for node in  self.graph.nodes:
            self.draw.draw_points([np.array(node.get_position())],[np.array(node.draw_color)],[node.draw_size])
        for edge in  self.graph.edges:
            self.draw.draw_lines([np.array(edge.node1.get_position())],[np.array(edge.node2.get_position())],[np.array(edge.draw_color)],[edge.draw_size])

    def draw_goal(self):
        if self.theia_1.current_node == None or self.forklift_1.current_node == None:
            return
        if self.theia_1.next_node == None or self.forklift_1.next_node == None:
            return
        if self.theia_1.goal_node == None or self.forklift_1.goal_node == None:
            return
        dot_color = (0, 1, 0, .5)
        dot_size  = (20)
        self.draw.draw_points([self.theia_1.goal_node.get_position()],[dot_color],[dot_size])
        dot_color = (1, 0, 0, .5)
        dot_size  = (20)
        self.draw.draw_points([self.forklift_1.goal_node.get_position()],[dot_color],[dot_size])

    def draw_simple_robots(self):
        if self.theia_1.current_node == None or self.forklift_1.current_node == None:
            return
        if self.theia_1.next_node == None or self.forklift_1.next_node == None:
            return
        if self.theia_1.goal_node == None or self.forklift_1.goal_node == None:
            return
        current = np.copy(self.theia_1.current_pose.get_position())
        current[2] = 2
        heading = np.copy(current)
        heading[0] += np.cos(self.theia_1.current_pose.get_heading_from_orientation()/180*np.pi) * 2
        heading[1] += np.sin(self.theia_1.current_pose.get_heading_from_orientation()/180*np.pi) * 2
        heading[2] = 4
        FOV1       = np.copy(current)
        FOV2       = np.copy(current)
        FOV1[2]    = 4
        FOV2[2]    = 4 
        FOV1[0] += np.cos((self.theia_1.current_pose.get_heading_from_orientation()+50)/180*np.pi) * 4
        FOV1[1] += np.sin((self.theia_1.current_pose.get_heading_from_orientation()+50)/180*np.pi) * 4
        FOV2[0] += np.cos((self.theia_1.current_pose.get_heading_from_orientation()-50)/180*np.pi) * 4
        FOV2[1] += np.sin((self.theia_1.current_pose.get_heading_from_orientation()-50)/180*np.pi) * 4
        self.draw.draw_lines([current],[FOV1],[(1,1,1,.5)],[2])
        self.draw.draw_lines([current],[FOV2],[(1,1,1,.5)],[2])
        self.draw.draw_lines([current],[heading],[(0,0,0,1)],[10])
        self.draw.draw_points([current],[(0,1,0,1)],[(40)])

        current = np.copy(self.forklift_1.current_pose.get_position())
        current[2] = 2
        heading = np.copy(current)
        heading[0] += np.cos(self.forklift_1.current_pose.get_heading_from_orientation()/180*np.pi) * 2
        heading[1] += np.sin(self.forklift_1.current_pose.get_heading_from_orientation()/180*np.pi) * 2
        heading[2] = 4
        self.draw.draw_points([current],[(1,0,0,1)],[(40)])
        self.draw.draw_lines([current],[heading],[(0,0,0,1)],[10])

    def draw_path(self):
        dot_color = (0, 1, 0, .5)
        dot_size  = (20)
        if self.theia_1.current_node == None or self.forklift_1.current_node == None:
            return
        if self.theia_1.next_node == None or self.forklift_1.next_node == None:
            return
        if self.theia_1.goal_node == None or self.forklift_1.goal_node == None:
            return
        
        self.draw.draw_lines([self.theia_1.current_pose.get_position()],[self.theia_1.next_node.get_position()],[dot_color],[5])
        prev_position = self.theia_1.next_node.get_position()
        for node in self.theia_1.global_plan:
            # self.draw.draw_points([node.get_position()],[dot_color],[dot_size])
            self.draw.draw_lines([node.get_position()],[prev_position],[dot_color],[5])
            prev_position = node.get_position()

        dot_color = (1, 0, 0, .5)
        dot_size  = (20)
        

        self.draw.draw_lines([self.forklift_1.current_pose.get_position()],[self.forklift_1.next_node.get_position()],[dot_color],[5])
        prev_position = self.forklift_1.next_node.get_position()
        for node in self.forklift_1.global_plan:
            # self.draw.draw_points([node.get_position()],[dot_color],[dot_size])
            self.draw.draw_lines([node.get_position()],[prev_position],[dot_color],[5])
            prev_position = node.get_position()



    def spin(self) -> None:
        self.world.pause()
        while self.kit.is_running():
            self.spin_once()

        omni.timeline.get_timeline_interface().stop()
        self.kit.close()

    def load_stage(self):
        #TODO: Parameterize usd_path
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/world/test_world_1.usd"

        # make sure the file exists before we try to open it
        try:
            result = is_file(usd_path)
        except:
            result = False

        if result:
            omni.usd.get_context().open_stage(usd_path)
        else:
            carb.log_error(
                f"the usd path {usd_path} could not be opened"
            )
            self.kit.close()
            sys.exit()

        print("Loading stage...")
        while is_stage_loading():
            self.kit.update()
        print("Loading Complete")        

    def log_collision_to_file(self, filename):
        t = time.localtime()
        file = open(filename,"a")
        write_string = str(self.world.current_time) + ", " + time.strftime("%H:%M:%S",t) + "\n"
        file.write(write_string)
        file.close

    def spin_once(self):
        self.draw_update()
        self.forklift_1.spin_once()
        self.theia_1.spin_once()

        if np.sqrt((self.theia_1.current_pose.get_position()[0]-self.forklift_1.current_pose.get_position()[0])**2+(self.theia_1.current_pose.get_position()[1]-self.forklift_1.current_pose.get_position()[1])**2) < 3:
            #TODO:Cleanup this implementation
            self.theia_1.clear_goal()
            self.theia_1.clear_next_node()
            # self.theia_1 = agent.Agent(self.graph,"/home/grntmtz/Desktop/qual_simulation/src/isaac_sim_theia/config/theia_robot.json",self.world)
            self.theia_1 = theia.Theia(self.graph,"src/isaac_sim_theia/config/theia_robot.json",self.world)
            self.theia_1.randomize_position()

            while self.theia_1.current_node == self.forklift_1.current_node: #making sure they aren't overlapping to start
                self.theia_1.randomize_position()

            self.theia_1.randomize_goal()
            while self.theia_1.goal_node == self.forklift_1.goal_node: #making sure that they aren't routing to the exact same node
                self.theia_1.randomize_goal()
            print("collision detected")
            self.log_collision_to_file("results/tdsp1.txt")

        self.kit.update()
        self.draw.clear_points()
        self.draw.clear_lines()
        
    
    