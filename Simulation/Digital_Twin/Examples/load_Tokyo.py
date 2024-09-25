import sys
import os

try:
    
    currentdir = os.path.dirname(os.path.abspath(__file__))  # Examples
    parentdir = Path(currentdir).parent.absolute()  # Digital_Twin
    sys.path.append(str(parentdir))

    from TokyoLoad import *
    from TokyoLoad import TokyoLoader

except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


# EXAMPLE USAGE
if __name__ == "__main__":

    my_world = World(stage_units_in_meters=1.0, backend="numpy")
    my_world.scene.add_default_ground_plane()

    usdLoader = TokyoLoader()
    usdLoader.import_world(existing_world=my_world)

    usdLoader.define_tokyo_usd_path()  # This is the only difference
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