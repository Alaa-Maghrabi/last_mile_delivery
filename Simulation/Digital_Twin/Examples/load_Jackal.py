import sys
import os
from pathlib import Path

currentdir = os.path.dirname(os.path.abspath(__file__))  # Examples
parentdir = Path(currentdir).parent.absolute()  # Digital_Twin
sys.path.append(str(parentdir))

try:
    from USDLoad import *
    from USDLoad import USDLoader
    
except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


if __name__ == "__main__":

    my_world = World(stage_units_in_meters=1.0, backend="numpy")
    my_world.scene.add_default_ground_plane()

    usdLoader = USDLoader()
    usdLoader.import_world(existing_world=my_world)
    
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()
    # Load jackal from omniverse library
    path_to_usd = assets_root_path + "/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd"  # Example to load robot
    
    # Load jackal from local directory
    # path_to_usd = "/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/Resources/Digital_Twins/Jackal/jackal_basic.usd"
    
    usdLoader.define_usd_path(path_to_usd=path_to_usd)
    usdLoader.define_usd_name(name_of_usd='Jackal')
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