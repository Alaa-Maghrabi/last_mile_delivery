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
    
    from Sensors.Camera.CameraLoading import *
    from Sensors.Camera.CameraLoading import CameraLoader
    
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
    
    #################### Camera
    camera = CameraLoader()
    camera.import_world(existing_world=example_world)

    camera_name = "CameraExample"
    camera.define_camera_name(camera_name=camera_name)

    # parent_name = "World"
    parent_name = jackal_prim_path + "/base_link/visuals/mesh_6"
    camera.define_camera_parent(parent=parent_name)

    # Below, the "random_key" is not loaded in the camera
    # NOTE: You are manually displacing camera to where you want it on robot
    camera_params = {
        "resolution": (256, 256),
        "translation": np.array([0.134, -0.0067, -0.121]),
        "random_key": 500.0,
    }
    
    camera.define_camera_parameters(cameraParams=camera_params)

    camera.create_camera()
    camera.world.reset()

    camera.initialise_camera(image_mode='rgb')

    ########### MAIN LOOP
    robot.world.reset()

    while simulation_app.is_running():
        robot.world.step(render=True)
        if robot.world.is_playing():
            if robot.world.current_time_step_index == 0:
                robot.world.reset()
                robot.controller.reset()

            robot.sim_step()
            captured_image = camera.get_camera_data()
            print(captured_image)
            cv2.imwrite('/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/scripts/tempimg.png', captured_image)


    print('I AAAAMA HEEEEEEREEEEEEEEEEE')

    simulation_app.close()
    
    # cv2.imwrite('/home/spyros/Elm/Code/DeliveryBot/last_mile_delivery/scripts/tempimg.png', captured_image)

    
    
if __name__ == "__main__":
    main()