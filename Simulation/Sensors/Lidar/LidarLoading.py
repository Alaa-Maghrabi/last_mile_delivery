import sys
import os

try:
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Lidar
    parentdir = Path(currentdir).parent.absolute()  # Sensors
    simulation_dir = Path(currentdir).parent.parent.absolute()  # Simulation
    sys.path.append(str(simulation_dir))

    from Master_Simluation_Loader import *

except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


class LidarLoader(BaseSample):

    def __init__(self) -> None:
        super().__init__()

        self.world = World()
        self.stage = None
        self.timeline = None

        self.lidar_prim = None
        self.lidar_parent = (
            "/World"  # keep like this if you don't want a parent for lidar
        )
        self.lidar_name = "/Lidar"
        self.complete_lidar_path = None

        self.lidar_result = None
        self.lidarParameters = {
            "path": self.lidar_parent + self.lidar_name,
            "parent": self.lidar_parent,
            "min_range": 0,
            "max_range": 100.0,
            "draw_points": True,
            "draw_lines": True,
            "horizontal_fov": 360.0,
            "vertical_fov": 60.0,
            "horizontal_resolution": 0.4,
            "vertical_resolution": 0.4,
            "rotation_rate": 1.0,
            "high_lod": True,
            "yaw_offset": 0.0,
            "enable_semantics": True,
        }

    def import_world(self, existing_world: omni.isaac.core.world.world.World):

        assert (
            type(existing_world) == omni.isaac.core.world.world.World
        ), "Ensure correct format of world passed. Aborting."
        self.world = existing_world

    def create_scene(self, path_to_physics_scene: str = "/World/PhysicsScene"):
        # Taken from https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_range_sensor_lidar.html
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface()
        )  # Used to interact with simulation
        omni.kit.commands.execute(
            "AddPhysicsSceneCommand", stage=self.stage, path=path_to_physics_scene
        )

    def define_lidar_parameters(self, lidarParams):

        for param in list(lidarParams.keys()):

            if param in list(self.lidarParameters.keys()):
                print(f'Updating parameter: {param}')
                self.lidarParameters[param] = lidarParams[param]

    def define_lidar_name(self, lidar_name: str):

        self.lidar_name = lidar_name

    def define_lidar_parent(self, parent: str):

        self.lidar_parent = parent

    def create_lidar(self, type_of_lidar: str = "RangeSensorCreateLidar"):

        # Ensure a '/' exists before the lidar name to complete the path
        if self.lidar_name[0] != "/":
            self.lidar_name = "/" + self.lidar_name

        # Repeat for parent
        if self.lidar_parent[0] != "/":
            self.lidar_parent = "/" + self.lidar_parent

        self.complete_lidar_path = self.lidar_parent + self.lidar_name

        self.lidarParameters["path"] = self.lidar_name
        self.lidarParameters["parent"] = self.lidar_parent

        self.lidar_result, self.lidar_prim = omni.kit.commands.execute(
            type_of_lidar, **self.lidarParameters
        )

    def translate_lidar(self, location: tuple):

        UsdGeom.XformCommonAPI(self.lidar_prim).SetTranslate(location)


# Example usage
if __name__ == "__main__":
    
    example_world = World()

    lidar = LidarLoader()
    lidar.import_world(existing_world=example_world)
    lidar.create_scene()

    lidar_name = "LidarExample"
    lidar.define_lidar_name(lidar_name=lidar_name)
    
    parent_name = 'World'
    lidar.define_lidar_parent(parent=parent_name)

    # Below, the "random_key" is not loaded in the lidar
    lidar_params = {
        "draw_points": True,
        "draw_lines": True,
        "vertical_fov": 30.0,
        "random_key": 500.0,
        "rotation_rate": 10.0
    }
    lidar.define_lidar_parameters(lidarParams=lidar_params)
    
    lidar.create_lidar(type_of_lidar="RangeSensorCreateLidar")
    
    starting_location = (-0.232, 0.0, 0.514)
    lidar.translate_lidar(location=starting_location)
    
    lidar.world.reset()
    
    while simulation_app.is_running():
        lidar.world.step(render=True)
        if lidar.world.is_playing():
            if lidar.world.current_time_step_index == 0:
                lidar.world.reset()

    simulation_app.close()
