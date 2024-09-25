import sys
import os

try:
    
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Examples
    parentdir = Path(currentdir).parent.absolute()  # Robot_Manipulation
    sys.path.append(str(parentdir))

    from Keyboard_Manipulation.WheeledKeyboardHoming import *
    from Keyboard_Manipulation.WheeledKeyboardHoming import KeyboardWheeledRobotHoming
    
    
except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


### EXAMPLE USAGES:

# 1) main_imported_params: shows how to import robot etc in class
# 2) main_unknown_params: shows how to setup class without params
# 3) main_mixed_params: checks that class works even with missing params


def main_defined():

    robot = KeyboardWheeledRobotHoming()

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
    # Homing Controller
    example_homing_controller = WheelBasePoseController(
        name="homing_controller",
        open_loop_wheel_controller=DifferentialController(
            name="homing_simple_control", wheel_radius=0.03, wheel_base=0.1125
        ),
        is_holonomic=False,
    )

    # Import params
    robot.import_world(example_world)
    robot.import_wheeled_robot(example_robot)
    robot.import_differential_controller(example_controller)
    robot.import_homing_controller(example_homing_controller)
    
    # Setup scene
    robot.setup_wheeled_robot_homing_scene()
    robot.setup_homing_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()
                robot.homing_controller.reset()

            robot.sim_homing_step()

    simulation_app.close()


def main_undefined():
    
    robot = KeyboardWheeledRobotHoming()
    
    # Setup scene
    robot.setup_wheeled_robot_homing_scene()
    robot.setup_homing_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()
                robot.homing_controller.reset()

            robot.sim_homing_step()

    simulation_app.close()


def main_mixed():
    
    robot = KeyboardWheeledRobotHoming()

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
    robot.setup_wheeled_robot_homing_scene()
    robot.setup_homing_keyboard()

    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()
                robot.homing_controller.reset()

            robot.sim_homing_step()

    simulation_app.close()


if __name__ == "__main__":

    main_defined()
    # main_undefined()
    # main_mixed()
