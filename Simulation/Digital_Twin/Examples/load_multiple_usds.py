import sys
import os
from pathlib import Path

currentdir = os.path.dirname(os.path.abspath(__file__))
parentdir = Path(currentdir).parent.absolute()
sys.path.append(str(parentdir))

try:
    from USDLoad import *
    from USDLoad import USDLoader
    
except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


def load_jackal(my_world):
    
    usdLoader_Jackal = USDLoader()
    usdLoader_Jackal.import_world(existing_world=my_world)

    dir_to_repo = Path(os.path.dirname(os.path.abspath(__file__))).parent.parent.parent.absolute()  # Takes you to repo dir
    path_to_usd = os.path.join(dir_to_repo, 'Resources', 'Digital_Twins', 'Jackal', 'jackal_basic.usd')
    # path_to_usd = "/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/Resources/Digital_Twins/Jackal/jackal_basic.usd"
    
    usdLoader_Jackal.define_usd_path(path_to_usd=path_to_usd)
    usdLoader_Jackal.define_usd_name(name_of_usd='Jackal')
    _ = usdLoader_Jackal.setup_usd_scene()
    
    return usdLoader_Jackal

def load_jetbot(my_world):
    
    usdLoader_Jetbot = USDLoader()
    usdLoader_Jetbot.import_world(existing_world=my_world)
    
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()
    path_to_usd = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"  # Example to load robot
    
    usdLoader_Jetbot.define_usd_path(path_to_usd=path_to_usd)
    usdLoader_Jetbot.define_usd_name(name_of_usd='Jetbot')
    _ = usdLoader_Jetbot.setup_usd_scene()
    
    return usdLoader_Jetbot
    
if __name__ == "__main__":

    my_world = World(stage_units_in_meters=1.0, backend="numpy")
    my_world.scene.add_default_ground_plane()

    _ = load_jackal(my_world=my_world)
    _ = load_jetbot(my_world=my_world)

    my_world.reset()

    while simulation_app.is_running():
        my_world.step(
            render=True
        )  # bit confused about how to use class variable or my_world
        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                my_world.reset()

    simulation_app.close()