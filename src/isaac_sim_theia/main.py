from omni.isaac.kit import SimulationApp

import cpp_mcl_acceleration
print("Works")
input()

CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}
#TODO: Create config json
kit = SimulationApp(launch_config=CONFIG)

import simulation_handler


if __name__ == "__main__":
    
    sim = simulation_handler.QualSimulationHandler(kit)
    sim.spin()
