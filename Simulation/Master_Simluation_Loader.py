import sys
import os

print(f'Importing main simulation from {os.path.abspath(__file__)}')
try:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    from omni.isaac.core.utils.extensions import enable_extension
    
    enable_extension("omni.isaac.examples")
    
    ###################################################
    ############## KEYBOARD ROBOTS ####################
    ###################################################
    
    import carb  # Used for input handling
    import omni.ext
    import omni.appwindow  # Contains handle to keyboard
    import gc
    import numpy as np
    from omni.isaac.wheeled_robots.robots import WheeledRobot
    from omni.isaac.wheeled_robots.controllers.differential_controller import (
        DifferentialController,
    )
    from omni.isaac.examples.base_sample import BaseSample
    from omni.isaac.core.utils.viewports import set_camera_view
    from omni.isaac.core.utils.nucleus import get_assets_root_path

    from omni.isaac.core import World

    # Controller
    from omni.isaac.core.utils.types import ArticulationAction
    from omni.isaac.core.controllers import BaseController

    # This extension includes several generic controllers that could be used with multiple robots
    from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import (
        WheelBasePoseController,
    )

    # Robot specific controller
    from omni.isaac.wheeled_robots.controllers.differential_controller import (
        DifferentialController,
    )
    
    
    ###################################################
    ################# USD LOADER ######################
    ###################################################
    import argparse
    import random
    import sys

    import carb
    import numpy as np
    import torch
    import omni

    from omni.isaac.cloner import Cloner
    from omni.isaac.core import World
    from omni.isaac.core.materials.omni_glass import OmniGlass
    from omni.isaac.core.prims.xform_prim_view import XFormPrimView
    from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
    from omni.isaac.core.utils.prims import define_prim
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.nucleus import get_assets_root_path
    from omni.isaac.core.prims import XFormPrim

    from omni.isaac.core.utils.viewports import set_camera_view
    from omni.isaac.core.utils.nucleus import get_assets_root_path

    from omni.isaac.core import World

    # Controller
    from omni.isaac.core.utils.types import ArticulationAction
    from omni.isaac.core.controllers import BaseController

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--test", default=False, action="store_true", help="Run in test mode"
    )
    args, unknown = parser.parse_known_args() 
    
    # Sensors
    ## Lidar
    from pxr import UsdGeom, Gf, UsdPhysics, Semantics              # pxr usd imports used to create cube
    
       
except ImportError as e:
    print(f'Import error in {os.path.basename(__file__)}')
    raise e