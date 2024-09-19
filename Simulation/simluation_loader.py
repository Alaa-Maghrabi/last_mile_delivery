import sys
import os

try:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    from omni.isaac.core.utils.extensions import enable_extension
    
except ImportError as e:
    print(f'Import error in {os.path.basename(__file__)}')
    raise e