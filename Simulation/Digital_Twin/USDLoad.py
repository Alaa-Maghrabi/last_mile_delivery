import sys
import os

try:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

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


except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


class USDLoader:
    def __init__(self) -> None:
        super().__init__()
        self.world = None

        self.usd_path = None
        self.usd_object = None
        self.usd_name = "Default_name"

    def import_world(self, existing_world: omni.isaac.core.world.world.World):

        assert (
            type(existing_world) == omni.isaac.core.world.world.World
        ), "Ensure correct format of world passed. Aborting."
        self.world = existing_world

    def confirm_values(self):

        if self.world is None:
            print("Initialising standard world.")
            self.world = World()

    def define_usd_path(self, path_to_usd: str):
        self.usd_path = path_to_usd
        
    def define_usd_name(self, name_of_usd: str):
        self.usd_name = name_of_usd

    def setup_usd_scene(self):

        self.confirm_values()  # setup world if not imported

        assert self.usd_path != None, "No path to usd provided. Aborting."
        
        self.usd_object = add_reference_to_stage(
            usd_path=self.usd_path, prim_path=f"/World/{self.usd_name}"
        )
        self.world.scene.add(XFormPrim(prim_path=f"/World/{self.usd_name}", name=f"{self.usd_name}"))

        return self.usd_object


## EXAMPLE USAGE

if __name__ == "__main__":

    # assets_root_path = get_assets_root_path()
    # if assets_root_path is None:
    #     carb.log_error("Could not find Isaac Sim assets folder")
    #     simulation_app.close()
    #     sys.exit()

    my_world = World(stage_units_in_meters=1.0, backend="numpy")
    my_world.scene.add_default_ground_plane()

    usdLoader = USDLoader()
    usdLoader.import_world(existing_world=my_world)

    path_to_usd = "/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/Resources/Digital_Twins/Jackal/jackal_basic.usd"
    
    usdLoader.define_usd_path(path_to_usd=path_to_usd)
    usdLoader.define_usd_name(name_of_usd='Example_name')
    _ = usdLoader.setup_usd_scene()

    my_world.reset()

    while simulation_app.is_running():
        my_world.step(
            render=True
        )  # bit confused about how to use class variable or my_world
        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                my_world.reset()

    simulation_app.close()
