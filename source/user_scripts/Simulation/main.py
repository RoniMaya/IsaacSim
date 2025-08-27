from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})

from InputManager import InputManager
from InputServer import InputServer
from InputMapper import InputMapper
import threading


import numpy as np,scipy


from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
import numpy as np

from Asset import Asset
from CameraClass import CameraClass
from Enviorment import Enviorment
from Controller import Controller
from RingBuffer import RingBuffer
from VideoPublisher import VideoPublisher
from DetectionPublisher import DetectionPublisher
import os
import time
from Radar import Radar

width = 640
height = 360
physics_frequency = 90  # Hz
rendering_frequency = 90  # Frames per second
render_every_n_frame = int(physics_frequency / rendering_frequency)
asset_speed = 80  # Speed of the asset in the simulation
delta_angle = 0.5
CFG_FILE  = '/home/ronim/isaacsim/source/user_scripts/Simulation/bindings.yaml'


imgr = InputManager()
input_server = InputServer(imgr)
imapper = InputMapper(CFG_FILE)

# Start background threads -----------------------------------------------------------
# start server thread - this will listen to controller inputs
threading.Thread(target=input_server.start, daemon=True).start()

video_rb = RingBuffer(capacity=8, drop_policy="latest")
VideoPublisher(video_rb)



radar_rb = RingBuffer(capacity=512, drop_policy="latest")

radar_pub = DetectionPublisher(ring_buffer=radar_rb, target_fps=1)
threading.Thread(target=radar_pub.start, daemon=True).start()



# open_stage("/home/ronim/Downloads/Showcases_Content_NVD@10011/Samples/Showcases/2023_2_1/IsaacWarehouse/IsaacWarehouse.usd")


# Add ogmar--------------------------------------------------------------------------------------------
# stage_path = os.path.join(os.path.expanduser("~"), "Documents", "cesium", "ogmar_at_origin.usd")
# open_stage(stage_path)

print("initilizing radar")
rcs_file_path = '/home/ronim/Documents/radar_sim/radar_rcs_maps/rcs_ball_1m_1.pkl'
radar_prop_path = '/home/ronim/Documents/radar_sim/radar_parameters/MAGOS.yaml'

delta_az = 80
delta_el = 30
radar_angle = [0, 90, 0]               
origin_world_radar = np.array([0, 0, 0])
radar = Radar(rcs_file_path, radar_prop_path, "ball", origin_world_radar, radar_angle, delta_az=delta_az, delta_el=delta_el)



world = World()
enviorment = Enviorment(world)
world.get_physics_context().set_gravity(0.0)

# default ground plane
world.scene.add_default_ground_plane()

DynamicCuboid(prim_path="/World/cube", color=np.array([0, 255, 0]))
cube = Asset("/World/cube")

DynamicCuboid(prim_path="/World/cube2", color=np.array([255, 0, 0]))
cube2 = Asset("/World/cube2")

VisualCuboid(prim_path="/World/cube_radar", color=np.array([0, 0, 255]))
cube_radar = Asset("/World/cube_radar", rigid_prim = False, geometry_prim = True)




cube_radar.set_pose(translation=origin_world_radar, orientation=None)
cube.set_pose(translation=np.array([0, 0, 50.0]), orientation = None)
cube2.set_pose(translation=np.array([5, 0, 0.0]), orientation = None)

# attatch a camera to the cube
camera = CameraClass(prim_path = "/World/cube/camera",orientation = np.array([0, 90, 0]),translation = [0,0,0.0],resolution = (width, height))
enviorment.add_dome_light(light_path="/dome_light",intensity= 1000)


world.reset()
camera.camera.initialize()


controller = Controller()
frame_count = 0
while simulation_app.is_running():
    if world.is_playing():

        # get pressed keys from the input manager and update the physics and camera
        pressed_keys = imgr.snapshot()
        mapping = imapper.calculate_mapping(pressed_keys)  # {'keyboard1': {'W','A',...}} -> {'throttle': 1.0, 'steering': -1.0, ...}

        # update velocity, orientation for each assets (+zoom for camera)__________________________________________
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        velocity = controller.update_velocity_direction(mapping, 'camera')
        camera_orientation = controller.update_orientation(mapping, 'camera')
        zoom_factor = controller.zoom_factor(mapping, 'camera')
        #----------------------------------------------------------------------------
        # set the velocity, orientation and zoom of the "camera" cube
        translation, orientation = cube.get_position_and_orientation()
        cube.set_angular_velocity_local([camera_orientation], orientation)
        cube.set_velocity_local(velocity, orientation)
        camera.zoom_camera(zoom_factor)
        #-------------------------------------------------------------------------


        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        translation, orientation = cube2.get_position_and_orientation()
        cube2_velocity = controller.update_velocity_direction(mapping, 'cube2')
        cube2_orientation = controller.update_orientation(mapping, 'cube2')
        #----------------------------------------------------------------------------
        # set the velocity, orientation and zoom of the "camera" cube
        cube2.set_angular_velocity_local([cube2_orientation], orientation)
        cube2.set_velocity_local(cube2_velocity, orientation)
        velocity = cube2.get_linear_velocity(orientation)
        #___________________________________________________________________________________________________________



        # if the frame count is divisible by the render_every_n_frame, define should_render == True----------
        should_render = (frame_count % render_every_n_frame == 0)
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True

        target, false_alarm = radar.get_detections(translation, orientation, velocity)
        range_target = target["range"] if target is not None else None
        range_false_alarm = false_alarm["range"] if false_alarm is not None else None
        detection = {"seq": frame_count, "time": time.time(), "range": range_target, "fa_range": range_false_alarm}


        radar_rb.push(detection)

        # if should_render is True, get the camera frame and add it to the queue for processing
        if should_render:
            frame_rgb_bytes = camera.frame_in_bytes()
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing

    frame_count += 1
