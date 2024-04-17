# generate ground truth map for EKF SLAM

import omni
from omni.isaac.core.utils.nucleus import is_file
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.stage as StageUtils
import omni.isaac.core.utils.prims as PrimUtils

# import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core import World

import carb 
import sys 

import cTheiaSLAM as c
import json
import os 

import numpy as np

class MapGenerator():

    def __init__(self, kit, json_path = None) -> None:
        self.kit = kit
        self.landmarks = []

        if json_path == None:
            print("JSON parameter file not provided, default settings used")
            self.run_default()
        elif( not os.path.isfile(json_path)):
            print("Config file at\n" + json_path + " \n is not found, default settings used")
            self.run_default()
        else:
            # self.load_config(json_path)
            #TODO: handle not default case (random generation)
            print("Currently not handled for non default case, WIP, running default")
            self.run_default()

    def run_default(self):
        json_path = os.path.dirname(__file__) + "/config/example_map_config.json"
        if not os.path.isfile(json_path):
            print("example map config file missing, generating example file")
            self.generate_default_config()
        else:
            self.load_config(json_path)

        if not os.path.isfile(self.usd_path) or self.regen == True:
            self.generate_default_map()
        else:
            self.load_stage_from_file()

    def load_config(self,json_path):
        f = open(json_path, 'r')
        self.__dict__.update(json.load(f))

    def generate_default_config(self):
        self.json_path = os.path.dirname(__file__) + "/config/example_map_config.json"
        self.regen = False
        self.usd_path = os.path.dirname(__file__) + "/maps/example_map.usd"
        
        self.generate_default_map()

        new_json = {k:v for (k,v) in self.__dict__.items() if self.json_key_filter(k)}
        with open(self.json_path, 'w') as f:
            json.dump(new_json, f, indent=4, sort_keys=True)

    def generate_default_map(self):
        print("Generating default map")
        self.world = World(stage_units_in_meters = 1.0)
        landmark_locations = [(3,6), (3,12), (7,8), (7,14), (11,6), (11,12)]
        i = 1
        for location in landmark_locations:

            self.landmarks.append(self.world.scene.add(
                VisualCuboid(
                    prim_path="/World/landmarks/landmark_" + str(i),
                    name="landmark_" + str(i),
                    position=np.array([location[0], location[1], 0.5]),
                    size=0.3,
                    color=np.array([255, 0, 0]),
                )
            ))

            i += 1

        self.world.scene.add_ground_plane()
        print("Done generating default map")
        self.save_map()

    def save_map(self):
        if not os.path.exists(os.path.dirname(__file__) + "/maps"):
            os.mkdir(os.path.dirname(__file__) + "/maps")
        if os.path.isfile(self.usd_path):
            print("USD file already exists, enter y to overwrite with current stage")
            if not input() == 'y':
                print("Exiting without saving")
                return
        StageUtils.save_stage(self.usd_path)
        print("Saved stage to " + self.usd_path)

    def load_stage_from_file(self):
        # make sure the file exists before we try to open it
        StageUtils.open_stage(self.usd_path)
        print("Loading stage...")
        while StageUtils.is_stage_loading():
            self.kit.update()
        print("Loading Complete")   

        self.world = World(stage_units_in_meters = 1.0)
        self.landmarks = PrimUtils.get_prim_children(PrimUtils.get_prim_at_path("/World/landmarks"))

    def json_key_filter(self, k):
        filter_list = ["json_path", "regen", "usd_path"]
        if k in filter_list:
            return True
        return False
        