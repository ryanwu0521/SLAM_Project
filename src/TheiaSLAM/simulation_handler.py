# handles all the Isaac Sim related functionality for EKF SLAM

import omni

from omni.isaac.core import World
from omni.isaac.debug_draw import _debug_draw
from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.stage as stage_utils


import numpy as np
import re
import math
from math import radians, sin, cos

import cTheiaSLAM as c 
from map_generator import MapGenerator
from cTheiaSLAM import *

 

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
        self.forklift_prim = self.load_forklift_usd()

        # set agent motion speed
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # update the simulation to apply changes
        self.kit.update()


    def sync_world_position(self, prim, position, orientation):
        prim.set_world_pose(position = position, orientation = orientation)
 

    def load_theia_usd(self):
        # define the path to the robot
        name = "Theia"
        usd_path = "omniverse://cerlabnucleus.lan.local.cmu.edu/Users/gmetts/theia_isaac_qual/robots/theia_robot/theia_robot.usd"
        prim_path = "/World/theia_robot"
        height_offset = 0.25
        position =[0.0, 0.0, height_offset]
        articulation_root = "/World/theia_robot/static/articulation_link"
        
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
        position = [0.0, 0.0, height_offset]
        articulation_root = "/World/forklift"
        
        # add the robot to the world
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        # create the robot prim
        forklift_pose = Pose(np.array(position), Rotation.from_euler('z', [0], True))
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


    '''tried to move agent using change in angle but it did not work (quaternion issue)'''
    def move_agent_controls(self, theia_dx, theia_dtheta):
        if hasattr(self, 'theia_prim'):
            # move theia
            theia_position = self.theia_prim.get_position()
            theia_orientation = self.theia_prim.get_orientation()

            # calculate the new position
            orientation_quat = Rotation.from_quat(theia_orientation)
            orientation_matrix = orientation_quat.as_matrix()
            forward_direction = np.dot(orientation_matrix, np.array([1, 0, 0]))
            new_theia_position = [theia_position[0] + theia_dx * forward_direction[0], 
                                  theia_position[1] + theia_dx * forward_direction[1], 
                                  theia_position[2] + theia_dx * forward_direction[2]]

            # calculate the new orientation
            # current_theia_orientation = list(orientation_quat.as_euler('xyz', degrees = False))
            # print("Current orientation before update:", current_theia_orientation)
            # current_theia_orientation[0] += theia_dtheta
            # current_theia_orientation[0] %= 2 * np.pi
            # new_theia_orientation = Rotation.from_euler('xyz', current_theia_orientation, degrees = False).as_quat()
            # print("New orientation:", new_theia_orientation)

            # calculate the new orientation
            current_euler = list(orientation_quat.as_euler('xyz', degrees = False)) 
            current_heading = current_euler[2]
            new_heading = current_heading + theia_dtheta
            new_theia_orientation= Rotation.from_euler('xyz', [0, 0, new_heading], degrees = False).as_quat()
            # new_orientation_quat = self.quaternion_from_axis_angle(axis, theia_dtheta)
            # new_theia_orientation = Rotation.multiply_quaternions(theia_orientation, new_orientation_quat)
            print("New orientation:", new_theia_orientation)

            # # update the theia prim position and orientation
            # self.theia_prim.set_position(new_theia_position)
            # self.theia_prim.set_orientation(new_theia_orientation)

            # update the pose using the Pose class helper functions
            theia_pose = Pose(np.array(new_theia_position), Rotation.from_quat(new_theia_orientation))
            self.theia_prim.set_position(theia_pose.get_position())
            self.theia_prim.set_orientation(theia_pose.get_orientation().as_quat())

            # update the theia world position
            self.sync_world_position(self.theia_prim, new_theia_position, new_theia_orientation)

            # generate the local trajectory
            self.generate_local_trajectory(new_theia_position)
            
            # update the simulation to apply changes
            self.kit.update()
        else:
            print("Theia and Forklift Prims not found")


    def calculate_turn_position(self, control_inputs):
        x, y, theta = 0, 0, 0
        positions = [(x, y)]

        for dx, dtheta in control_inputs:
            dx_new = dx * math.cos(theta)
            dy_new = dx * math.sin(theta)
            x += dx_new
            y += dy_new
            theta += dtheta
            positions.append((x, y))

        return positions


    def move_theia_to_position(self, theia_dx, theia_dy):
        if hasattr(self, 'theia_prim'):
            # move theia
            theia_position = self.theia_prim.get_position()
            theia_orientation = self.theia_prim.get_orientation()

            # calculate the change in x and y
            theia_dx_new = theia_dx - theia_position[0]
            theia_dy_new = theia_dy - theia_position[1]

            # calculate the new position
            new_theia_position = [
                theia_position[0] + theia_dx_new,
                theia_position[1] + theia_dy_new,
                theia_position[2]
            ]

            # update the theia prim position and orientation
            self.theia_prim.set_position(new_theia_position)

            # update the theia world position
            self.sync_world_position(self.theia_prim, new_theia_position, theia_orientation)

            # generate the local trajectory
            self.generate_local_trajectory(new_theia_position)

            # update the simulation to apply changes
            self.kit.update()
        else:
            print("Theia Prim not found")


    def move_forklift_to_position(self, forklift_dx, forklift_dy):
        if hasattr(self, 'forklift_prim'):
            # move forklift
            forklift_position = self.forklift_prim.get_position()
            forklift_orientation = self.forklift_prim.get_orientation()

            # calculate the change in x and y
            forklift_dx_new = forklift_dx - forklift_position[0]
            forklift_dy_new = forklift_dy - forklift_position[1]

            # calculate the new position
            new_forklift_position = [
                forklift_position[0] + forklift_dx_new,
                forklift_position[1] + forklift_dy_new,
                forklift_position[2]
            ]

            # update the forklift prim position and orientation
            self.forklift_prim.set_position(new_forklift_position)

            # update the forklift world position
            self.sync_world_position(self.forklift_prim, new_forklift_position, forklift_orientation)

            # generate the local trajectory
            self.generate_local_trajectory(new_forklift_position)

            # update the simulation to apply changes
            self.kit.update()
        else:
            print("Forklift Prim not found")


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
    
        # calculate the time required to reach the waypoint
        time_to_waypoint = distance_to_waypoint / linear_speed

        # set the robot's orientation to the direction of movement
        new_orientation = Rotation.from_euler('z', [angle_to_waypoint], degrees = False).as_quat()
        self.theia_prim.set_orientation(new_orientation)

        # Return the calculated trajectory
        return distance_to_waypoint, angle_to_waypoint, time_to_waypoint


    # def visualize_ekf_estimated_landmarks(self, estimated_landmarks):
    #         draw = _debug_draw.acquire_debug_draw_interface()
    #         # visualize the estimated landmarks
    #         for x, y in estimated_landmarks:
    #             position = Vector3(x, y, 0.0)
    #             scale = Vector3(0.1, 0.1, 0.1)
    #             color = [0, 1, 0, 1]  # green color
    #             lifetime = 1.0
    #             depth = 0.0
    #             draw.add_cuboid(position, scale, color, lifetime, depth)
    #         self.kit.update()
        

    def run_simulation(self):
        # data inputs for the theia agent
        theia_bearing_values, theia_range_values, theia_dx_values, theia_dtheta_values = self.read_data_from_file('data\data.txt')
        # data inputs for the forklift agent
        forklift_bearing_values, forklift_range_values, forklift_dx_values, forklift_dtheta_values = self.read_data_from_file('data\gen_data.txt')

        # print the control inputs
        # print("Theia Control Inputs:")
        # print("dx values:", theia_dx_values)
        # print("dtheta values:", theia_dtheta_values)

        # print("Forklift Control Inputs:")
        # print("dx values:", forklift_dx_values)
        # print("dtheta values:", forklift_dtheta_values)

        # read the control inputs for the theia agent
        theia_control_inputs = zip (theia_dx_values, theia_dtheta_values)
        # theia_measurement_inputs = zip( theia_bearing_values, theia_range_values)

        # read the control inputs for the forklift agent
        forklift_control_inputs = zip (forklift_dx_values, forklift_dtheta_values)

        # calculate the theia turning points based on the control inputs
        theia_turning_points = self.calculate_turn_position(theia_control_inputs)

        # calculate the forklift turning points based on the control inputs
        forklift_turning_points = self.calculate_turn_position(forklift_control_inputs)

        '''Move both agents at the same time'''
        # iterate through the maximum number of steps for either agent
        # max_steps = max(len(theia_turning_points), len(forklift_turning_points))
        # for step in range(max_steps):
        #     # move Theia 
        #     if step < len(theia_turning_points):
        #         theia_dx, theia_dy = theia_turning_points[step]
        #         print('Moving Theia to x, y:', theia_dx, theia_dy)
        #         self.move_theia_to_position(theia_dx, theia_dy)
        #         print("Theia's position:", self.theia_prim.get_position())

        #     # move Forklift
        #     if step < len(forklift_turning_points):
        #         forklift_dx, forklift_dy = forklift_turning_points[step]
        #         print('Moving Forklift to x, y:', forklift_dx, forklift_dy)
        #         self.move_forklift_to_position(forklift_dx, forklift_dy)
        #         print("Forklift's position:", self.forklift_prim.get_position())

        #     # Update the simulation after both agents have moved
        #     self.kit.update()

        # print("Theia and Forklift have reached the end of the control inputs")

        '''Move Theia first then Forklift'''
        for theia_dx, theia_dy in theia_turning_points:
            print('Moving Theia to x, y: ', theia_dx, theia_dy)
            # move the theia agent
            self.move_theia_to_position(theia_dx, theia_dy)
            # self.move_agent_xy(theia_dx, theia_dtheta)

            # print the agent position and orientation after moving
            print("Theia's position:", self.theia_prim.get_position())
            # print("Theia's orientation:", self.theia_prim.get_orientation())

        for forklift_dx, forklift_dy in forklift_turning_points:
            print('Moving Forklift to x, y: ', forklift_dx, forklift_dy)
            # move the forklift agent
            self.move_forklift_to_position(forklift_dx, forklift_dy)
            # self.move_agent_xy(theia_dx, theia_dtheta)

            # print the agent position and orientation after moving
            print("Forklift's position:", self.forklift_prim.get_position())
            # print("Theia's orientation:", self.theia_prim.get_orientation())

        # for bearing, range in theia_measurement_inputs:
        #     print('Moving Theia with measurement inputs: ', bearing, range)
        #     # move the theia agent
        #     self.move_agent_measurements(bearing, range)
            
        #     # print the agent's position and orientation after moving
        #     print("Theia's position:", self.theia_prim.get_position())
        #     print("Theia's orientation:", self.theia_prim.get_orientation())
            
            # time.sleep(1)
            self.kit.update()
        
        print("Theia and Forklift have reached the end of the control inputs")

        # add the estimated landmarks to the simulation
        # estimated_landmarks = [(2.988, 6.031),
        #                        (3.010, 12.034),
        #                        (7.015, 7.981),
        #                        (7.006, 14),
        #                        (10.996, 5.997),
        #                        (11.032, 11.969)]
        # self.visualize_ekf_estimated_landmarks(estimated_landmarks)


    def run(self) -> None:
        # main loop to run the simulation
        while self.kit.is_running():
            self.run_simulation()

        omni.timeline.get_timeline_interface().stop()
        self.kit.close()

# commendline to run the simulation:
# python isaac_python.py