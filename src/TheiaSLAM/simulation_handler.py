# handles all the Isaac Sim related functionality for EKF SLAM

import omni

from omni.isaac.core import World
from omni.isaac.debug_draw import _debug_draw
from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.stage as stage_utils


import numpy as np
import carb 

import cTheiaSLAM as c 
from map_generator import MapGenerator
from cTheiaSLAM import *

import agent
import time 

import multi_agent_ekf

class CustomXFormPrim(XFormPrim):
    def set_position(self, position):
        self.position = position

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

        # set up physics context
        self.physx = self.world.get_physics_context()
        self.physx.set_gravity(0.0)
        self.world.initialize_physics()

        omni.timeline.get_timeline_interface().play()

        # load the agents into the world  
        self.theia_prim = self.load_theia_usd()
        self.forklift_prim = self.load_forklift_usd()

        # update the simulation to apply changes
        self.kit.update()
        self.kit.update()
        self.kit.update()

    def sync_world_position(self, prim, position, orientation):
        prim.set_world_pose(position = position, orientation = orientation)

    def load_theia_usd(self):
        # define the path to the robot
        name = "Theia"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/theia_robot/theia_robot.usd"
        prim_path = "/World/theia_robot"
        height_offset = 0.25
        position = [0, 0, height_offset]
        articulation_root = "/World/theia_robot/static/articulation_link"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        prim = CustomXFormPrim(articulation_root, name = name, position = position)
        
        return prim
    
        
    def load_forklift_usd(self):
         # define the path to the robot
        name = "Forklift"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/forklift/forklift.usd"
        prim_path = "/World/forklift"
        height_offset = 0.25
        position = [10, 0, height_offset]
        articulation_root = "/World/forklift"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        prim = CustomXFormPrim(articulation_root, name = name, position = position, orientation = [0, 0, 0, 1])
        
        return prim
    
    def move_agent(self, theia_new_position, forklift_new_position):
        print ("Moving Theia to: ", theia_new_position)
        print ("Moving Forklift to: ", forklift_new_position)

        if hasattr(self, 'theia_prim') and hasattr(self, 'forklift_prim'):
            # move theia
            self.sync_world_position(self.theia_prim, theia_new_position, [0, 0, 0, 1])
           
            # move forklift
            self.sync_world_position(self.forklift_prim, forklift_new_position, [0, 0, 0, 1])

            # update the simulation to apply changes
            self.kit.update()
        else: 
            print("Theia and Forklift Prims not found")

    def run(self) -> None:
        # main loop to run the simulation
        while self.kit.is_running():
            self.run_simulation()

        omni.timeline.get_timeline_interface().stop()
        self.kit.close()

    # def log_bearing_range_data(self, theia_current_position, forklift_current_position,filename):
    #     landmark_positions = self.map_generator.landmark_positions

    #     file = open(filename,"a")
        
    #     for landmark_position in landmark_positions:
    #         # calculate the bearing and range
    #         bearing = np.arctan2(landmark_position[1] - theia_current_position[1], landmark_position[0] - theia_current_position[0])
    #         range = np.sqrt((landmark_position[1] - theia_current_position[1])**2 + (landmark_position[0] - theia_current_position[0])**2)

    #         # write the bearing and range to the file
    #         file.write(str(bearing) + "," + str(range) + "\n")

    #     file.close

    def run_simulation(self):
        # run a sinle step of the simulation
        theia_new_position = [10, 0, 0.25]
        forklift_new_position = [0, 0, 0.25]
        # print("Moving Theia to: " + str(theia_new_position))
        # print("Moving Forklift to: " + str(forklift_new_position))
        self.move_agent(theia_new_position, forklift_new_position)
        # print("Simulation running...")
        self.kit.update()
        # print("Simulation updated")