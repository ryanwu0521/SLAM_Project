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

        # load the agents into the world  
        self.load_Theia_usd()
        self.load_forklift_usd()
         
        # self.prim = XFormPrim()

        self.kit.update()
        self.kit.update()
        self.kit.update()

    def sync_world_pose(self):
        self.prim.set_world_pose(position = self.current_pose.get_position(), orientation=self.current_pose.get_quat_scalar_first())

    def load_Theia_usd(self):
        # define the path to the robot
        name = "Theia"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/theia_robot/theia_robot.usd"
        prim_path = "/World/theia_robot"
        height_offset = 0.25
        position = [0, 0, height_offset]
        # translation = [0, 0, 0]
        articulation_root = "/World/theia_robot/static/articulation_link"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        prim = XFormPrim(articulation_root, name = name, position = position)
        

        return prim
    
        
    def load_forklift_usd(self):
         # define the path to the robot
        name = "Forklift"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/forklift/forklift.usd"
        prim_path = "/World/forklift"
        height_offset = 0.25
        position = [15, 15, height_offset]
        # translation = [0, 0, 0]
        articulation_root = "/World/forklift"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        prim = XFormPrim(articulation_root, name = name, position = position, orientation = [0, 0, 0, 1])
        
        return prim

    # def move_robot(self, dx, dtheta):
    #     # get the current pose of the robot
    #     current_pose = self.slam_agent.current_pose
    #     current_position = current_pose.get_position()
    #     current_heading = current_pose.get_heading()

    #     if self.slam_agent is not None:
    #         # update the position of the robot
    #         new_position = current_position + np.array([dx, 0, 0])
    #         new_heading = current_heading + dtheta

    #         # set the new position of the robot
    #         self.slam_agent.set_position(new_position, new_heading)
    #     else:
    #         print("SLAM agent not initialized. Cannot move robot.")

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

        # self.move_robot(dx=3, dtheta=0.5)
        self.kit.update()

    
    