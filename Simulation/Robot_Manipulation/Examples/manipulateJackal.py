import sys
import os

try:
    
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Examples
    parentdir = Path(currentdir).parent.absolute()  # Robot_Manipulation
    sys.path.append(str(parentdir))

    from Keyboard_Manipulation.JackalKeyboardHoming import *
    from Keyboard_Manipulation.JackalKeyboardHoming import KeyboardJackalRobotHoming
    
    
except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


### EXAMPLE USAGES:

# 1) main_imported_params: shows how to import robot etc in class

# NOTE: Controller parameters for Jackal found here:
# https://forums.developer.nvidia.com/t/how-to-drive-clearpath-jackal-via-ros2-messages-in-isaac-sim/275907

def main_nohoming():

    robot = KeyboardJackalRobot()

    # Create parameters
    # World
    example_world = World()
    # Robot
    assets_root_path = get_assets_root_path()
    jackal_asset_path = assets_root_path + "/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd"
    example_robot = WheeledRobot(
        prim_path="/World/JackalBot",
        name="JackalBot",
        wheel_dof_names=["front_left_wheel", "front_right_wheel"],
        create_robot=True,
        usd_path=jackal_asset_path,
    )
    #wheel_dof_names=["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"],
    # Controller
    example_controller = DifferentialController(
        name="simple_control", wheel_radius=0.098, wheel_base=0.37558
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


def main_homing():

    robot = KeyboardJackalRobotHoming()

    # Create parameters
    # World
    example_world = World()
    # Robot
    assets_root_path = get_assets_root_path()
    jackal_asset_path = assets_root_path + "/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd"
    example_robot = WheeledRobot(
        prim_path="/World/JackalBot",
        name="JackalBot",
        wheel_dof_names=["front_left_wheel", "front_right_wheel"],
        create_robot=True,
        usd_path=jackal_asset_path,
    )
    #wheel_dof_names=["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"],
    # Controller
    example_controller = DifferentialController(
        name="simple_control", wheel_radius=0.098, wheel_base=0.37558
    )  # Create the Controller
    # Homing Controller
    example_homing_controller = WheelBasePoseController(
        name="homing_controller",
        open_loop_wheel_controller=DifferentialController(
            name="homing_simple_control", wheel_radius=0.098, wheel_base=0.37558
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



if __name__ == "__main__":

    # main_nohoming()
    main_homing()