import sys
import os

try:
    from pathlib import Path


    currentdir = os.path.dirname(os.path.abspath(__file__))
    parentdir = Path(currentdir).parent.absolute()  # Robot_manipulation
    simulation_dir = Path(currentdir).parent.parent.absolute()  # Simulation
    sys.path.append(str(simulation_dir))
    
    from Master_Simluation_Loader import *
    
except ImportError as e:
    print(f'Import error in {os.path.basename(__file__)}')
    raise e

# loosely based on:
# https://forums.developer.nvidia.com/t/how-to-use-keyboard-control-a-robot-in-a-python-standalong-application/202829/2


class KeyboardWheeledRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.world = None
        self.robot_type = None

        self.controller = None

        self._command = [0.0, 0.0]  # Default Actions (No Motion)

    def import_world(
        self,
        existing_world: omni.isaac.core.world.world.World
    ):

        assert (
            type(existing_world)
            == omni.isaac.core.world.world.World
        ), "Ensure correct format of world passed. Aborting."
        self.world = existing_world

    def import_wheeled_robot(
        self, robot: omni.isaac.wheeled_robots.robots.wheeled_robot.WheeledRobot
    ):

        assert (
            type(robot) == omni.isaac.wheeled_robots.robots.wheeled_robot.WheeledRobot
        ), "Ensure correct Robot format. Aborting."
        self.robot_type = robot

    def import_differential_controller(
        self,
        controller: omni.isaac.wheeled_robots.controllers.differential_controller.DifferentialController,
    ):

        assert (
            type(controller)
            == omni.isaac.wheeled_robots.controllers.differential_controller.DifferentialController
        ), "Ensure correct Controller format. Aborting."
        self.controller = controller

    def confirm_values(self):

        if self.world is None:
            print("Initialising standard world.")
            self.world = World()

        if self.robot_type is None:
            print("Initialising Jetbot, standard wheeled robot.")

            assets_root_path = get_assets_root_path()
            jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
            self.robot_type = WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
            )

        if self.controller is None:

            print("Initialising standard controller.")

            self.controller = DifferentialController(
                name="simple_control", wheel_radius=0.03, wheel_base=0.1125
            )  # Create the Controller

    def setup_wheeled_robot_scene(self):

        self.confirm_values()  # Create custom values if no param imported
        
        self.world.scene.add_default_ground_plane()  # MAYBE REMOVE
        self.jetbot = self.world.scene.add(self.robot_type)

        print("Keyboard Robot Scene Prepared")

    def setup_keyboard(self):

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()  # Grab the Input handle
        self._keyboard = self._appwindow.get_keyboard()  # Get the Keyboard
        self._sub_keyboard = (
            self._input.subscribe_to_keyboard_events(  # Subscribe to Keyboard Events
                self._keyboard, self._sub_keyboard_event
            )
        )

        print("Keyboard Commands Initialised")

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """_summary_

        Args:
            event (_type_): _description_

        Returns:
            _type_: _description_
        """

        # print(f'RECEIVED COMMAND: {self._command}')

        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
        ):
            if (
                event.input == carb.input.KeyboardInput.W
            ):  # Set the Action Command for each Wheel
                self._command = [20, 0.0]
            if event.input == carb.input.KeyboardInput.S:
                self._command = [-20, 0.0]
            if event.input == carb.input.KeyboardInput.A:
                self._command = [0.0, np.pi / 5]
            if event.input == carb.input.KeyboardInput.D:
                self._command = [0.0, -np.pi / 5]

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._command = [0.0, 0.0]

        return True

    def sim_step(self):  # When Simulating

        self.jetbot.apply_wheel_actions(  # Send the Actions to the Controller
            self.controller.forward(command=self._command)
        )


### EXAMPLE USAGES:

# 1) main_imported_params: shows how to import robot etc in class
# 2) main_unknown_params: shows how to setup class without params
# 3) main_mixed_params: checks that class works even with missing params


def main_imported_params():

    robot = KeyboardWheeledRobot()

    # Create parameters
    # World
    example_world = World()
    # Robot
    assets_root_path = get_assets_root_path()
    jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
    example_robot = WheeledRobot(
        prim_path="/World/Fancy_Robot",
        name="fancy_robot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
    )
    # Controller
    example_controller = DifferentialController(
        name="simple_control", wheel_radius=0.03, wheel_base=0.1125
    )  # Create the Controller

    # Import params
    robot.import_world(example_world)
    robot.import_wheeled_robot(example_robot)
    robot.import_differential_controller(example_controller)
    
    # Setup scene
    robot.setup_wheeled_robot_scene()
    robot.setup_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()

            robot.sim_step()

    simulation_app.close()


def main_unknown_params():
    
    robot = KeyboardWheeledRobot()
    
    # Setup scene
    robot.setup_wheeled_robot_scene()
    robot.setup_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()

            robot.sim_step()

    simulation_app.close()


def main_mixed_params():
    
    robot = KeyboardWheeledRobot()

    # Create parameters
    # World
    example_world = World()
    # Controller
    example_controller = DifferentialController(
        name="simple_control", wheel_radius=0.03, wheel_base=0.1125
    )  # Create the Controller

    # Import params
    robot.import_world(example_world)
    robot.import_differential_controller(example_controller)
    
    # Setup scene
    robot.setup_wheeled_robot_scene()
    robot.setup_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()

            robot.sim_step()

    simulation_app.close()


if __name__ == "__main__":

    # main_imported_params()
    # main_unknown_params()
    main_mixed_params()
