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


def setup_ros2_lidar_publisher(lidar_prim_path):
    """ Set up ROS2 bridge for streaming LIDAR data. """
    keys = og.Controller.Keys

    # Define the OmniGraph path and ROS2 nodes
    graph_path = "/ActionGraph"
    lidar_topic = "/jackal/lidar"

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("LidarPublisher", "omni.isaac.ros2_bridge.ROS2LidarHelper"),
            ],
            keys.CONNECT: [
                # Connect the context to the lidar publisher
                ("Context.outputs:context", "LidarPublisher.inputs:context"),
            ],
            keys.SET_VALUES: [
                # Set ROS2 topic and prim path for the LIDAR publisher
                ("LidarPublisher.inputs:frameId", "sim_lidar"),
                ("LidarPublisher.inputs:topicName", lidar_topic),
                ("LidarPublisher.inputs:sensorPrimPath", lidar_prim_path),
                ("Context.inputs:domain_id", 1),
            ],
        },
    )

    return graph_path

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

    # Set up ROS2 LIDAR publisher using OmniGraph
    lidar_ros2_graph = setup_ros2_lidar_publisher(lidar_prim_path=f"{jackal_prim_path}/base_link/visuals/mesh_6/LidarExample")    

    ########### MAIN LOOP
    robot.world.reset()
    frame = 0

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()


            # Trigger ROS2 publishing every 5 frames
            og.Controller.attribute(lidar_ros2_graph + "/LidarPublisher.inputs:step").set(frame % 5 == 0)

            robot.sim_step()
            frame = frame + 1

    simulation_app.close()
    
    
if __name__ == "__main__":
    main()