from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt
import cv2

my_world = World(stage_units_in_meters=1.0)

camera = Camera(
    prim_path="/World/camera",  # try to mount on robot by using the path to the mounting spot
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

my_world.scene.add_default_ground_plane()
my_world.reset()
camera.initialize()

i = 0
camera.add_motion_vectors_to_frame()

while simulation_app.is_running():
    my_world.step(render=True)
    print(camera.get_current_frame())
    # Check https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html
    image = camera.get_rgb()
    # image = camera.get_rgba()  # In case you want rgba
    print(f'Type of image: {type(image)}')
    print(image.shape)
    
    if i == 100:
        
        # cv2.imshow(f"rgb ({i})", image) 
        # cv2.imshow(f"depth ({i})", depth)
        cv2.waitKey(1)
        
        # imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        # plt.show()
        # print(camera.get_current_frame()["motion_vectors"])
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
    i += 1
    
simulation_app.close()