import sys
import os

try:
    
    currentdir = os.path.dirname(os.path.abspath(__file__))  # Digital_Twin
    sys.path.append(currentdir)

    from USDLoad import *
    from USDLoad import USDLoader

except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


class TokyoLoader(USDLoader):
    def __init__(self) -> None:
        super().__init__()

    def define_tokyo_usd_path(self, path_to_tokyo_usd: str = None):
        """_summary_
        If None is passed, the code uses the hardcoded path below.

        Args:
            path_to_tokyo_usd (str, optional): _description_. Defaults to None.
        """
        # TODO: Tokyo usd seems to have a scale factor of 100 (e.g. 80 meters become 8000 meters)


        if path_to_tokyo_usd is None:
            self.usd_path = "omniverse://localhost/Projects/Tokyo_Complete/TOK.usd"
        else:
            self.usd_path = path_to_tokyo_usd
            
        self.define_usd_name(name_of_usd='Tokyo')


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
