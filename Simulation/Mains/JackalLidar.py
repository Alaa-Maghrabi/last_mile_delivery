import sys
import os

try:
    
    from pathlib import Path

    currentdir = os.path.dirname(os.path.abspath(__file__))  # Mains
    parentdir = Path(currentdir).parent.absolute()  # Simulation
    sys.path.append(str(parentdir))

    from Robot_Manipulation.Keyboard_Manipulation.JackalKeyboard import *
    from Robot_Manipulation.Keyboard_Manipulation.JackalKeyboard import KeyboardJackalRobot
    
    from Digital_Twin.TokyoLoad import *
    from Digital_Twin.TokyoLoad import TokyoLoader
    
    from Sensors.Lidar.LidarLoading import *
    from Sensors.Lidar.LidarLoading import LidarLoader
    
except ImportError as e:
    print(f"Import error in {os.path.basename(__file__)}")
    raise e


def main():
    
    ############# ROBOT
    robot = KeyboardJackalRobot()

    # Create parameters
    # World
    example_world = World()
    # Robot
    assets_root_path = get_assets_root_path()
    jackal_asset_path = assets_root_path + "/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd"
    jackal_prim_path = "/World/JackalBot"
    example_robot = WheeledRobot(
        prim_path=jackal_prim_path,
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
    
    #################### Lidar
    lidar = LidarLoader()
    lidar.import_world(existing_world=example_world)
    lidar.create_scene()

    lidar_name = "LidarExample"
    lidar.define_lidar_name(lidar_name=lidar_name)
    
    # Note, for Jackal, the lidar should be placed at 
    # /World/JackalBot/base_link/visuals/mesh_6
    parent_name = jackal_prim_path + "/base_link/visuals/mesh_6"
    lidar.define_lidar_parent(parent=parent_name)

    # Below, the "random_key" is not loaded in the lidar
    lidar_params = {
        "draw_points": True,
        "draw_lines": True,
        "vertical_fov": 30.0,
        "random_key": 500.0,
        "rotation_rate": 0.5
    }
    lidar.define_lidar_parameters(lidarParams=lidar_params)
    
    lidar.create_lidar(type_of_lidar="RangeSensorCreateLidar")
    
    # starting_location = (-0.232, 0.0, 0.514)
    starting_location = (0.0, 0.0, 0.02)  # Manually displace lidar from 0 0 0
    lidar.translate_lidar(location=starting_location)

    ########### MAIN LOOP
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
    main()