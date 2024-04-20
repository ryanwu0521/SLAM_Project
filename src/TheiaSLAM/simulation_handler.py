# handles all the Isaac Sim related functionality for EKF SLAM

import omni

from omni.isaac.core import World
from omni.isaac.debug_draw import _debug_draw
from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.core.prims import XFormPrim
# from omni.isaac.core import omni_math
import omni.isaac.core.utils.stage as stage_utils


import numpy as np
import time
import carb 
import re

import cTheiaSLAM as c 
from map_generator import MapGenerator
from cTheiaSLAM import *

import agent
import multi_agent_ekf
 

class CustomXFormPrim(XFormPrim):
    def __init__(self, articulation_root, name, position, orientation) -> None:
        super().__init__(articulation_root, name = name, position = position, orientation = orientation)
        self.position = position
        self.orientation = orientation

    def set_position(self, position):
        self.position = position

    def get_position(self):
        return self.position
    
    def get_orientation(self):
        return self.orientation
    
    def set_orientation(self, orientation):
        self.orientation = orientation

class SLAMSimulationHandler:

    def __init__(self, kit) -> None:
        #TODO: Add in functionality to choose world
        self.kit = kit
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.map_generator = MapGenerator(kit)

        # set up the world
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
        # self.forklift_prim = self.load_forklift_usd()

        # set agent motion speed
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # update the simulation to apply changes
        self.kit.update()


    def sync_world_position(self, prim, position, orientation):
        prim.set_world_pose(position = position, orientation = orientation)
 

    def load_theia_usd(self):
        # define the path to the robot
        # name = "Theia"
        # usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/theia_robot/theia_robot.usd"
        # prim_path = "/World/theia_robot"
        # height_offset = 0.25
        # position =[0.0, 0.0, height_offset]
        # articulation_root = "/World/theia_robot/static/articulation_link"

        name = "Forklift"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/forklift/forklift.usd"
        prim_path = "/World/forklift"
        height_offset = 0.25
        position = [0.0, 0.0, height_offset]
        articulation_root = "/World/forklift"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        theia_pose = Pose(np.array(position), Rotation.from_euler('z', [0], True))
        theia_prim = CustomXFormPrim(articulation_root, name = name, position = theia_pose.get_position(), orientation = theia_pose.get_quat_scalar_first())
        
        return theia_prim
    
        
    def load_forklift_usd(self):
         # define the path to the robot
        name = "Forklift"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/forklift/forklift.usd"
        prim_path = "/World/forklift"
        height_offset = 0.25
        position = [15.0, 0.0, height_offset]
        articulation_root = "/World/forklift"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        forklift_pose = Pose(np.array(position), Rotation.from_euler('z', [180], True))
        forklift_prim = CustomXFormPrim(articulation_root, name = name, position = forklift_pose.get_position(), orientation = forklift_pose.get_quat_scalar_first())

        return forklift_prim
    

    def read_data_from_file(self, file_path):
        # initialize the measurement and control input lists
        bearing_values = []
        range_values = []
        dx_values = []
        dtheta_values = []  # radians 

        # read the measurement and control inputs from the file
        with open(file_path, 'r') as file:    
            lines = file.readlines()

            # loop for bearing and range values
            for i in range(0, len(lines), 2):
                fields = re.split('[\t ]', lines[i])[:-1]
                bearing = float(fields[0])
                r = float(fields[1])

                bearing_values.append(bearing)
                range_values.append(r)

            # loop for dx and dtheta values
            for i in range(1, len(lines), 2):
                fields = re.split('[\t ]', lines[i])[:-1]
                dx = float(fields[0])
                dtheta = float(fields[1])

                dx_values.append(dx)
                dtheta_values.append(dtheta)  

        return bearing_values, range_values, dx_values, dtheta_values


    def move_agent_controls(self, theia_dx, theia_dtheta):
        if hasattr(self, 'theia_prim'):
            # move theia
            theia_position = self.theia_prim.get_position()
            theia_orentation = self.theia_prim.get_orientation()

            # calculate the new position
            orientation_quat = Rotation.from_quat(theia_orentation)
            orientation_matrix = orientation_quat.as_matrix()
            forward_direction = np.dot(orientation_matrix, np.array([1, 0, 0]))
            new_theia_position = [theia_position[0] + theia_dx * forward_direction[0], theia_position[1] + theia_dx * forward_direction[1], theia_position[2]]

            # calculate the new orientation
            current_theia_orientation = orientation_quat.as_euler('xyz', degrees = False)
            current_theia_orientation[0] += theia_dtheta
            new_theia_orientation = Rotation.from_euler('xyz', current_theia_orientation, degrees = False).as_quat()

            # update the theia prim position and orientation
            self.theia_prim.set_position(new_theia_position)
            self.theia_prim.set_orientation(new_theia_orientation)

            # update the theia world position
            self.sync_world_position(self.theia_prim, new_theia_position, new_theia_orientation)

            # generate the local trajectory
            # self.generate_local_trajectory(new_theia_position)
            
            # update the simulation to apply changes
            self.kit.update()
        else:
            print("Theia and Forklift Prims not found")


    def generate_local_trajectory(self, new_position):
        # calculate the local trajectory for the agent
        current_position = self.theia_prim.get_position()
        dx = new_position[0] - current_position[0]
        dy = new_position[1] - current_position[1]

        # calculate the angle to the next waypoint
        angle_to_waypoint = np.arctan2(dy, dx)

        # calculate the distance to the next waypoint
        distance_to_waypoint = np.sqrt(dx**2 + dy**2)

        # set the linear and angular speeds
        linear_speed = self.linear_speed
        angular_speed = self.angular_speed

        # calculate the time required to reach the waypoint
        time_to_waypoint = distance_to_waypoint / linear_speed

        # calculate the change in orientation required to align with the waypoint
        angle_diff = angle_to_waypoint - self.theia_prim.get_orientation()

        # calculate the angular speed required to turn to the waypoint
        angular_speed_to_waypoint = angle_diff / time_to_waypoint

        # set the robot's orientation to the direction of movement
        new_orientation = Rotation.from_euler('z', [angle_to_waypoint], degrees = False).as_quat()
        self.theia_prim.set_orientation(new_orientation)

        # Return the calculated trajectory
        return distance_to_waypoint, angle_to_waypoint, angular_speed_to_waypoint


    def visualize_ekf_estimated_landmarks(self, estimated_landmarks):
            # visualize the estimated landmarks
            for landmark in estimated_landmarks:
                # draw the estimated landmark
                self.draw.add_cuboid(landmark, [0.1, 0.1, 0.1], [1, 0, 0, 1], 1.0, 0.0)
            
            # update the simulation to apply changes
            self.kit.update()
    

    def run_simulation(self):
        # data inputs for the theia agent
        bearing_values, range_values, dx_values, dtheta_values = self.read_data_from_file('src\TheiaSLAM\data\data.txt')
        
        # print the control inputs
        # print('bearing values:', bearing_values)
        # print('range values:', range_values)
        print('dx values:', dx_values)
        print('dtheta values:', dtheta_values)

        # create the control inputs for the pentagon trajectory
        theia_control_inputs = zip (dx_values, dtheta_values)

        # flag to check if there are more control inputs
        has_more_control_inputs = True
        
        for i, (theia_dx, theia_dtheta) in enumerate(theia_control_inputs):
            print('Moving Theia with control inputs: ', theia_dx, theia_dtheta)
            
            # move the theia agent
            self.move_agent_controls(theia_dx, theia_dtheta)

            # print the theia agent's position and orientation after moving
            print("Theia's position:", self.theia_prim.get_position())
            print("Theia's orientation:", self.theia_prim.get_orientation())
            
            time.sleep(1)

            # check if there are more control inputs
            if i == len(dx_values) - 1:
                has_more_control_inputs = False
                break

        if not has_more_control_inputs:
            print("Theia has reached the end of the control inputs")
            
            
        self.kit.update()


    def run(self) -> None:
        # main loop to run the simulation
        while self.kit.is_running():
            self.run_simulation()

        omni.timeline.get_timeline_interface().stop()
        self.kit.close()