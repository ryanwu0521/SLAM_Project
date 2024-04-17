# handles all the Isaac Sim related functionality for EKF SLAM

import omni

from omni.isaac.core import World
from omni.isaac.debug_draw import _debug_draw

import numpy as np
import carb 

import cTheiaSLAM as c 
from map_generator import MapGenerator

import agent
import time 

import multi_agent_ekf

class SLAMSimulationHandler:

    def __init__(self, kit) -> None:
        #TODO: Add in functionality to choose world
        self.kit = kit
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.map_generator = MapGenerator(kit)


        self.world = self.map_generator.world
        self.world.set_simulation_dt(physics_dt = 0.5, rendering_dt = 0.5) #TODO: Parameterize

        self.landmarks = self.map_generator.landmarks
        # self.graph = self.map_generator.graph

        # set sim's physics context
        self.physx = self.world.get_physics_context()
        self.physx.set_gravity(0.0)
        self.world.initialize_physics()

        omni.timeline.get_timeline_interface().play()

        self.sig_x = 0.25
        self.sig_y = 0.1
        self.sig_alpha = 0.1
        self.sig_beta = 0.01
        self.sig_r = 0.08

        self.sig_x2 = self.sig_x**2
        self.sig_y2 = self.sig_y**2
        self.sig_alpha2 = self.sig_alpha**2
        self.sig_beta2 = self.sig_beta**2
        self.sig_r2 = self.sig_r**2
        
        self.control_cov = np.diag([self.sig_x2, self.sig_y2, self.sig_alpha2])
        self.measure_cov = np.diag([self.sig_beta2, self.sig_r2])


        self.slam_agent_theia = multi_agent_ekf.Agent("src/TheiaSLAM/data/data.txt", self.control_cov, self.measure_cov, "src/TheiaSLAM/config/theia_robot.json", self.world) 

        self.slam_agent.set_init_position_to_pose([0,0,0],[0,0,0,1])

        # self.slam_agent.randomize_position()
        # self.slam_agent.randomize_goal()

        self.kit.update()
        self.kit.update()
        self.kit.update()
     

    def draw_update(self):
        self.draw_graph()
        self.draw_path()
        self.draw_goal()
    
    def draw_graph(self):
        for node in  self.graph.nodes:
            self.draw.draw_points([np.array(node.get_position())],[np.array(node.draw_color)],[node.draw_size])
        for edge in  self.graph.edges:
            self.draw.draw_lines([np.array(edge.node1.get_position())],[np.array(edge.node2.get_position())],[np.array(edge.draw_color)],[edge.draw_size])

    def draw_goal(self):
        if self.slam_agent.current_node == None or self.slam_agent.next_node == None or self.slam_agent.goal_node == None:
            return
        dot_color = (0, 1, 0, .5)
        dot_size  = (20)
        self.draw.draw_points([self.slam_agent.goal_node.get_position()],[dot_color],[dot_size])

    def draw_path(self):
        dot_color = (0, 1, 0, .5)
        if self.slam_agent.current_node == None or self.slam_agent.next_node == None or self.slam_agent.goal_node == None:
            return
        
        self.draw.draw_lines([self.slam_agent.current_pose.get_position()],[self.slam_agent.next_node.get_position()],[dot_color],[5])
        prev_position = self.slam_agent.next_node.get_position()
        for node in self.slam_agent.global_plan:
            self.draw.draw_lines([node.get_position()],[prev_position],[dot_color],[5])
            prev_position = node.get_position()

    def spin(self) -> None:
        # self.world.pause()
        while self.kit.is_running():
            self.spin_once()

        omni.timeline.get_timeline_interface().stop()
        self.kit.close()

    def log_collision_to_file(self, filename):
        t = time.localtime()
        file = open(filename,"a")
        write_string = str(self.world.current_time) + ", " + time.strftime("%H:%M:%S",t) + "\n"
        file.write(write_string)
        file.close

    def spin_once(self):

        # self.slam_agent.spin_once()
        self.kit.update()

    
    