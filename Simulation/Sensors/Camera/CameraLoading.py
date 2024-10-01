import sys
import os

try:
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Camera
    parentdir = Path(currentdir).parent.absolute()  # Sensors
    simulation_dir = Path(currentdir).parent.parent.absolute()  # Simulation
    sys.path.append(str(simulation_dir))

    from Master_Simluation_Loader import *
    
    import omni.isaac.core.utils.numpy.rotations as rot_utils
    import numpy as np
    import matplotlib.pyplot as plt
    import cv2


except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


class CameraLoader(BaseSample):
    # https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/Documentation/Isaac-Sim-Docs_2022.2.1/isaacsim/latest/isaac_sim_sensors_camera.html


    def __init__(self) -> None:
        super().__init__()

        self.world = World()

        self.camera_prim = None
        self.camera_parent = (
            "/World"  # keep like this if you don't want a parent for camera
        )
        self.camera_name = "/Camera"
        self.complete_camera_path = None
        
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html
        self.cameraParameters = {
            "prim_path": self.camera_parent + self.camera_name,
            "name": self.camera_name,
            "frequency": None,
            "dt": None,
            "resolution": (256, 256),
            "orientation": rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            "render_product_path": None
        }
        
        self.captured_image = None  # Maybe initialise as np array of size = camera resolution
        self.image_types = ['rgb', 'rgba']
        self.image_capture = None


    def import_world(self, existing_world: omni.isaac.core.world.world.World):

        assert (
            type(existing_world) == omni.isaac.core.world.world.World
        ), "Ensure correct format of world passed. Aborting."
        self.world = existing_world
        

    def define_camera_parameters(self, cameraParams):

        for param in list(cameraParams.keys()):

            if param in [list(self.cameraParameters.keys()), 'translation', 'position']:
                print(f"Updating parameter: {param}")
                self.cameraParameters[param] = cameraParams[param]

    def define_camera_name(self, camera_name: str):

        self.camera_name = camera_name

    def define_camera_parent(self, parent: str):

        self.camera_parent = parent

    def create_camera(self):

        # Ensure a '/' exists before the camera name to complete the path
        if self.camera_name[0] != "/":
            self.camera_name = "/" + self.camera_name

        # Repeat for parent
        if self.camera_parent[0] != "/":
            self.camera_parent = "/" + self.camera_parent

        self.complete_camera_path = self.camera_parent + self.camera_name

        self.cameraParameters["name"] = self.camera_name
        self.cameraParameters["prim_path"] = self.camera_parent + self.camera_name

        print(f"Created camera with the following parameters: \n")
        print(self.cameraParameters)

        self.camera_prim = Camera(**self.cameraParameters)
        
        # make sure you do camera_primm.initialize() after this method
        return self.camera_prim

    def initialise_camera(self, image_mode: str = 'rgb'):
        self.camera_prim.initialize()
        self.camera_prim.add_motion_vectors_to_frame()
        
        assert image_mode in self.image_types, f"Choose image from {self.image_types}. Aborting."
        
        if image_mode == 'rgb':
            self.image_capture = self.camera_prim.get_rgb
        elif image_mode == 'rgba':
            self.image_capture = self.camera_prim.get_rgba

        
    def get_camera_data(self):
        
        self.captured_image = self.image_capture()
        return self.captured_image



if __name__ == "__main__":
    
    example_world = World()

    camera = CameraLoader()
    camera.import_world(existing_world=example_world)

    camera_name = "CameraExample"
    camera.define_camera_name(camera_name=camera_name)

    parent_name = "World"
    camera.define_camera_parent(parent=parent_name)

    # Below, the "random_key" is not loaded in the camera
    camera_params = {
        "resolution": (256, 256),
        "position": np.array([0.0, 0.0, 25.0]),
        "random_key": 500.0,
    }
    
    camera.define_camera_parameters(cameraParams=camera_params)

    camera.create_camera()
    camera.initialise_camera(image_mode='rgb')
    
    camera.world.reset()

    while simulation_app.is_running():
        camera.world.step(render=True)
        camera.camera_prim.get_current_frame()
        if camera.world.is_playing():
            if camera.world.current_time_step_index == 0:
                camera.world.reset()

            captured_image = camera.get_camera_data()
            # print(captured_image)
            cv2.imwrite('/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/scripts/tempimg.png', captured_image)


    # FOR SOME REASON THIS SCRIPT AND ANY OTHER CAMERA ONE THROWS AN ERROR WHEN YOU TERMINATE SIM
    simulation_app.close()
