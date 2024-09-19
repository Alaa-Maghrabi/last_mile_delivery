# import isaacsim

# from isaacsim import SimulationApp

# simulation_app = SimulationApp({"headless": False})

# from omni.isaac.core.utils.extensions import enable_extension

# enable_extension("omni.isaac.examples")

# import omni
# import carb
# from omni.isaac.core import World
# from omni.isaac.examples.base_sample import BaseSample
# from omni.isaac.core.utils.nucleus import get_assets_root_path
# from omni.isaac.core.utils.types import ArticulationAction
# from omni.isaac.core.utils.nucleus import find_nucleus_server
# from omni.isaac.core.utils.stage import add_reference_to_stage
# from omni.isaac.core.prims import XFormPrim

# import omni.isaac.lab.sim as sim_utils
# from omni.isaac.core import SimulationContext

# import numpy as np
# import time


# class LoadJapan(BaseSample):
#     def __init__(self) -> None:
#         super().__init__()
#         return
    
#     def setup_scene(self):
#         world = self.get_world()
#         # world = World()
#         world.scene.add_default_ground_plane()
#         assets_root_path = get_assets_root_path()
#         # Method taken from: https://forums.developer.nvidia.com/t/how-to-add-my-own-usd-into-isaac-sim/251205
#         japan_usd_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"  # Example to load robot
#         # japan_usd_path = assets_root_path + "/Isaac/Desktop/Nishishinjuku_Japan_flat_01.usd"
#         # japan_usd_path = "home/spyros/Elm/Digital_Twins/Tokyo_Complete/TOK.usd"

#         # Currently, importing the Japanese model only works via drag and drop
#         add_reference_to_stage(usd_path=japan_usd_path, prim_path="/World/Log")
#         world.scene.add(XFormPrim(prim_path="/World/Log", name="log"))
        
#         return


# def main():
#     """Main function."""

#     simulation_app.update()

#     # Create light source
#     omni.kit.commands.execute(
#         "CreatePrim", prim_type="DomeLight", attributes={"inputs:intensity": 1000, "inputs:texture:format": "latlong"}
#     )

#     simulation_app.update()

#     tokyo_loader = LoadJapan()
#     tokyo_loader.setup_scene()

#     while simulation_app.is_running():
#         simulation_app.update()

#     simulation_app.close()
#     ####################################################################################################################
#     # # Initialize the simulation context
#     # sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))
#     # # Set main camera
#     # sim.set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])
#     # design scene
#     # tokyo_loader = LoadJapan()
#     # tokyo_loader.setup_scene()
#     # # Play the simulator
#     # sim.reset()
#     # # Now we are ready!
#     # print("[INFO]: Setup complete...")
#     # # Run the simulator
#     # while simulation_app.is_running():
    
#     #     sim.step()


# if __name__ == "__main__":

#     main()
#     # simulation_app.close()

# ############################################################################################################################################
# ############################################################################################################################################

# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import random
import sys

import carb
import numpy as np
import torch
from omni.isaac.cloner import Cloner
from omni.isaac.core import World
from omni.isaac.core.materials.omni_glass import OmniGlass
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim


parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0, backend="numpy")
my_world.scene.add_default_ground_plane()
# num_objects = 3
my_cloner = Cloner()

# Use add_reference_to_stage to tune translation, scale, etc:
# https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html

# The absolute path and the assets_root_path methods do not work for tokyo usd
# asset_path = "home/spyros/Elm/Digital_Twins/Tokyo_Complete/Nishishinjuku_Japan_01.usd"
# asset_path = assets_root_path + "/Isaac/Tokyo_Complete/TOK.usd"
# asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"  # Example to load robot

asset_path = "omniverse://localhost/Projects/Tokyo_Complete/TOK.usd"
# Tokyo usd seems to have a scale factor of 100 (e.g. 80 meters become 8000 meters)
stand = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Log")
stand.scale = np.array([0.01, 0.01, 0.01])
my_world.scene.add(XFormPrim(prim_path="/World/Log", name="log"))


# asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
# root_path = "/World/Group"
# root_group_path = root_path + "_0"
# group_paths = my_cloner.generate_paths(root_path, num_objects)
# define_prim(prim_path=root_group_path)
# add_reference_to_stage(usd_path=asset_path, prim_path=root_group_path + "/Franka")


# define_prim(root_group_path + "/Frame")
# define_prim(root_group_path + "/Frame/Target")
# my_cloner.clone(root_group_path, group_paths)

# frankas_view = XFormPrimView(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Franka", name="frankas_view")
# targets_view = XFormPrimView(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Frame/Target", name="targets_view")
# frames_view = XFormPrimView(prim_paths_expr=f"/World/Group_[0-{num_objects-1}]/Frame", name="frames_view")

# glass_1 = OmniGlass(
#     prim_path=f"/World/franka_glass_material_1",
#     ior=1.25,
#     depth=0.001,
#     thin_walled=False,
#     color=np.array([random.random(), random.random(), random.random()]),
# )

# glass_2 = OmniGlass(
#     prim_path=f"/World/franka_glass_material_2",
#     ior=1.25,
#     depth=0.001,
#     thin_walled=False,
#     color=np.array([random.random(), random.random(), random.random()]),
# )
# # new_positions = torch.tensor([[10.0, 10.0, 0], [-40, -40, 0], [40, 40, 0]])
# # new_orientations = euler_angles_to_quats(
# #     torch.tensor([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0], [0, 0, -np.pi / 2.0]])
# # )

# new_positions = np.array([[10.0, 10.0, 0], [-40, -40, 0], [40, 40, 0]])
# new_orientations = euler_angles_to_quats(np.array([[0, 0, np.pi / 2.0], [0, 0, -np.pi / 2.0], [0, 0, -np.pi / 2.0]]))

# frankas_view.set_world_poses(positions=new_positions, orientations=new_orientations)
# frankas_view.apply_visual_materials(visual_materials=glass_1, indices=[1])
# frankas_view.apply_visual_materials(visual_materials=[glass_1, glass_2], indices=[2, 0])
# print(frankas_view.get_applied_visual_materials(indices=[2, 0]))
# print(frankas_view.get_applied_visual_materials())

my_world.reset()

for i in range(10000):
    my_world.step(render=True)
simulation_app.close()
