import sys
import os

try:
    
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Digital_Twin
    simulation_dir = Path(currentdir).parent.absolute()  # Simulation
    sys.path.append(str(simulation_dir))
    
    from Master_Simluation_Loader import *


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
