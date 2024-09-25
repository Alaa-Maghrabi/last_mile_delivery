import sys
import os

try:
    
    currentdir = os.path.dirname(os.path.abspath(__file__))  # Keyboard_Manipulation
    sys.path.append(currentdir)

    # The way INHERITANCE works in Isaac is shown below. Basically, in order to import packages 
    # you need to run simulation_app = SimulationApp({"headless": False}) in the file. 
    # However, you cannot have more than one simulations running, and when you import 
    # the parent class, you automatically start a simulation. Therefore, ensure all 
    # packages of interest are in the parent class. Not sure how well this works with multiple files.
    from JackalKeyboard import *
    from JackalKeyboard import KeyboardJackalRobot
    
except ImportError as e:
    print(f'Import error in {os.path.basename(__file__)}')
    raise e

# loosely based on:
# https://forums.developer.nvidia.com/t/how-to-use-keyboard-control-a-robot-in-a-python-standalong-application/202829/2


class KeyboardJackalRobotHoming(KeyboardJackalRobot):
    def __init__(self) -> None:
        super().__init__()
        self.homing_controller = None
        self._homing_flag = False

    def import_homing_controller(
        self,
        homing_controller: omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller.WheelBasePoseController,
    ):

        assert (
            type(homing_controller)
            == omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller.WheelBasePoseController
        ), "Ensure correct Hominh Controller format. Aborting."
        self.homing_controller = homing_controller

    def confirm_homing_values(self):

        if self.world is None:
            print("Initialising standard world.")
            self.world = World()

        if self.robot_type is None:
            print("Initialising Jackal, standard wheeled robot.")

            assets_root_path = get_assets_root_path()
            jackal_asset_path = assets_root_path + "/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd"
            self.robot_type = WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="fancy_robot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jackal_asset_path,
            )

        if self.controller is None:

            print("Initialising standard controller.")

            self.controller = DifferentialController(
                name="simple_control", wheel_radius=0.098, wheel_base=0.37558
            )  # Create the Controller

        if self.homing_controller is None:

            print("Initialising standard homing controller.")

            self.homing_controller = WheelBasePoseController(
                name="homing_controller",
                open_loop_wheel_controller=DifferentialController(
                    name="homing_simple_control", wheel_radius=0.098, wheel_base=0.37558
                ),
                is_holonomic=False,
            )

    def setup_wheeled_robot_homing_scene(self):

        self.confirm_homing_values()  # Create custom values if no param imported
        
        self.world.scene.add_default_ground_plane()  # MAYBE REMOVE
        self.jackal = self.world.scene.add(self.robot_type)

        print("Keyboard Robot Scene Prepared")

    def setup_homing_keyboard(self):

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()  # Grab the Input handle
        self._keyboard = self._appwindow.get_keyboard()  # Get the Keyboard
        self._sub_keyboard = (
            self._input.subscribe_to_keyboard_events(  # Subscribe to Keyboard Events
                self._keyboard, self._sub_keyboard_homing_event
            )
        )

        print("Keyboard Commands Initialised")

    def _sub_keyboard_homing_event(self, event, *args, **kwargs):
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
                self._command = [7, 0.0]
            if event.input == carb.input.KeyboardInput.S:
                self._command = [-7, 0.0]
            if event.input == carb.input.KeyboardInput.A:
                self._command = [0.0, np.pi / 2]
            if event.input == carb.input.KeyboardInput.D:
                self._command = [0.0, -np.pi / 2]

            # Test spacebar option and generally different naming convention
            # This is the same method as the quadruped example
            # Make it turn left faster for testing

            # Here we add a homing mechanism that sets the flag to T or F
            # and uses the differential controller to return to 0, 0
            if event.input.name == "ENTER":
                if self._homing_flag is False:
                    print("Homing Initialised and Running...")
                    self._homing_flag = True
                else:
                    print("Homing interrupted")
                    self._homing_flag = False

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._command = [0.0, 0.0]

        return True

    def sim_homing_step(self):  # When Simulating

        # print('SIM STEP REACHED')

        if self._homing_flag is False:
            self.jackal.apply_wheel_actions(  # Send the Actions to the Controller
                self.controller.forward(command=self._command)
            )
        else:
            position, orientation = self.jackal.get_world_pose()
            print(f'Position: {position}')
            print(f'Orientation: {orientation}')
            self.jackal.apply_action(
                self.homing_controller.forward(
                    start_position=position,
                    start_orientation=orientation,
                    goal_position=np.array([0.0, 0.0]),
                )
            )
