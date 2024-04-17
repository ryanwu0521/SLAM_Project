# calls the simlationhandler and run the program

from omni.isaac.kit import SimulationApp

# run with Isaac Sim visualization
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

# run without Isaac Sim visualization
# CONFIG = {"sync_loads": True, "headless": True}

#TODO: Create config json
kit = SimulationApp(launch_config=CONFIG)

import simulation_handler


if __name__ == "__main__":
    
    sim = simulation_handler.SLAMSimulationHandler(kit)
    sim.spin()
