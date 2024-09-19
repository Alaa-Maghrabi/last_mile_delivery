from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.examples")

import carb                                                                     # Used for input handling
import omni.ext
import omni.appwindow                                                           # Contains handle to keyboard
import gc
import numpy as np
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.nucleus import get_assets_root_path

from omni.isaac.core import World

# Controller
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController
# This extension includes several generic controllers that could be used with multiple robots
from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
# Robot specific controller
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController


class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
    

class KeyboardRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.controller = None
        self.homing_controller = None
        self._command = [0.0, 0.0]  # Default Actions (No Motion)
        self._homing_flag = False
        self.world = World()

    def setup_scene(self):
        self.world.scene.add_default_ground_plane()  # Get the World
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self.jetbot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )
        )
        
        self.controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)  # Create the Controller

        self.homing_controller = WheelBasePoseController(name="homing_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="homing_simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125),
                                                        is_holonomic=False)
        
        print('SCENE PREPARED')

    def setup_post_load(self):
        # world = self.get_world()  # Get the World
        # self._jetbot = world.scene.get_object("my_jetbot")  # Get the Robot from the World
        
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()  # Grab the Input handle
        self._keyboard = self._appwindow.get_keyboard()  # Get the Keyboard
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(  # Subscribe to Keyboard Events
            self._keyboard, self._sub_keyboard_event
            )
        
        print('POST SCENE PREPARED')
        # self._world.add_physics_callback("jetbot_step", callback_fn=self._on_sim_step)  # Add Callback to Physics Step
        
        # Initialize our controller after load and the first reset
        
        
        # await self._world.play_async()  # Start the Simulation
                                                        

    # async def setup_pre_reset(self):
    #     self._controller.reset()  # resets the controller
    #     self._world.remove_physics_callback("jetbot_step") # Removes the physics callback
    #     return

    # async def setup_post_reset(self):
    #     self._world.add_physics_callback("jetbot_step", callback_fn=self._on_sim_step) # Adds the physics callback
    #     await self._world.play_async()
    #     return
    
    def _sub_keyboard_event(self, event, *args, **kwargs):

        print(f'RECEIVED COMMAND: {self._command}')

        if (event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT):
            if event.input == carb.input.KeyboardInput.W:  # Set the Action Command for each Wheel
                self._command = [20, 0.0]
            if event.input == carb.input.KeyboardInput.S:
                self._command = [-20, 0.0]
            if event.input == carb.input.KeyboardInput.A:
                self._command = [0.0, np.pi / 5]
            if event.input == carb.input.KeyboardInput.D:
                self._command = [0.0, -np.pi / 5]

            # Test spacebar option and generally different naming convention
            # This is the same method as the quadruped example
            # Make it turn left faster for testing

            # Here we add a homing mechanism that sets the flag to T or F 
            # and uses the differential controller to return to 0, 0
            if event.input.name == "ENTER":
                if self._homing_flag is False:
                    print('Homing Initialised and Running...')
                    self._homing_flag = True
                else:
                    print('Homing interrupted')
                    self._homing_flag = False

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._command = [0.0, 0.0]
        
        return True

    def sim_step(self):  # When Simulating

        print('SIM STEP REACHED')
        
        if self._homing_flag is False:
            self.jetbot.apply_wheel_actions(   # Send the Actions to the Controller
                self.controller.forward(command=self._command)
                )
        else:
            position, orientation = self.jetbot.get_world_pose()
            self.jetbot.apply_action(self.homing_controller.forward(start_position=position,
                                                                start_orientation=orientation,
                                                                goal_position=np.array([0.0, 0.0])))

if __name__ == "__main__":   

    robot = KeyboardRobot()
    robot.setup_scene()
    robot.setup_post_load()

    # my_world = robot.world
    # my_controller = robot.controller
    # my_homing_controller = robot.homing_controller
    # my_jetbot = robot.jetbot
 
    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()
                robot.homing_controller.reset()

            # my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
            robot.sim_step()

    simulation_app.close()
