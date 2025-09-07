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
import Utils
import os
import time
from Radar import Radar
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux,PhysxSchema, Sdf, Vt

import omni.usd
from scipy.interpolate import splprep, splev
import paho.mqtt.client as mqtt
import cv2

width = 1090
height = 720
physics_frequency = 120  # Hz
rendering_frequency = 30  # Frames per second
detection_frequency = 1  # Hz

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
VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread

radar_rb = RingBuffer(capacity=512, drop_policy="latest")
radar_pub = DetectionPublisher(ring_buffer=radar_rb, target_fps=1)
threading.Thread(target=radar_pub.start, daemon=True).start()


# Add ogmar--------------------------------------------------------------------------------------------
stage_path = os.path.join(os.path.expanduser("~"), "Documents", "cesium","textured_model", "z_upv2.usd")
tree_asset_path = "http://127.0.0.1:8080/omniverse://127.0.0.1/NVIDIA/Assets/Vegetation/Trees/Colorado_Spruce.usd"
car_asset_path = "/home/ronim/Documents/assets/car/car_v7.usd"
van_asset_path = "/home/ronim/Documents/assets/car/car_v9_rough.usd"

tree_path = "/World/tree"
car1_path = "/World/car1"
car2_path = "/World/car2"



cesium_transformation = Utils.cesium_transformation('/home/ronim/Documents/cesium/ogmar/conf.JSON')
# find camera position from real world coordinates
camera_position_dms = {'lon':[35,22,35.36,'E'], 'lat':[30,55,45.36,'N'],'height':-276}#,'lon':[35,22,35.57,'E'], 'lat':[30,55,45.43,'N'],'height':-270}
transformed_vertices = Utils.get_mesh_position_from_dms(camera_position_dms, cesium_transformation)


scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([tree_path, car1_path, car2_path],
                                                                                            [tree_asset_path, car_asset_path, van_asset_path])}
open_stage(stage_path)


spline_position_dms = [
 {'lon':[35,22,22.48,'E'], 'lat':[30,55,36.76,'N'],'height':-293},
 {'lon':[35,22,24.54,'E'], 'lat':[30,55,37.81,'N'],'height':-295},
 {'lon':[35,22,26.22,'E'], 'lat':[30,55,38.69,'N'],'height':-296},
 {'lon':[35,22,27.17,'E'], 'lat':[30,55,39.39,'N'],'height':-306},
 {'lon':[35,22,28.37,'E'], 'lat':[30,55,40.06,'N'],'height':-306},
 {'lon':[35,22,29.10,'E'], 'lat':[30,55,40.79,'N'],'height':-307},
 {'lon':[35,22,29.29,'E'], 'lat':[30,55,41.49,'N'],'height':-307},
 {'lon':[35,22,29.60,'E'], 'lat':[30,55,42.09,'N'],'height':-308},
 {'lon':[35,22,30.21,'E'], 'lat':[30,55,42.58,'N'],'height':-308},
 {'lon':[35,22,30.72,'E'], 'lat':[30,55,43.04,'N'],'height':-303},
 {'lon':[35,22,32.02,'E'], 'lat':[30,55,43.25,'N'],'height':-303},
 {'lon':[35,22,33.74,'E'], 'lat':[30,55,43.52,'N'],'height':-308},
 {'lon':[35,22,34.97,'E'], 'lat':[30,55,43.69,'N'],'height':-305}]


# spline_position_dms = [
#  {'lon':[35,22,22.48,'E'], 'lat':[30,55,36.76,'N'],'height':-293},
#  {'lon':[35,22,24.54,'E'], 'lat':[30,55,37.81,'N'],'height':-295},
#  {'lon':[35,22,26.22,'E'], 'lat':[30,55,38.69,'N'],'height':-296},
#  {'lon':[35,22,27.17,'E'], 'lat':[30,55,39.39,'N'],'height':-306},
#  {'lon':[35,22,28.37,'E'], 'lat':[30,55,40.06,'N'],'height':-306},
#  {'lon':[35,22,29.10,'E'], 'lat':[30,55,40.79,'N'],'height':-307},
#  {'lon':[35,22,29.29,'E'], 'lat':[30,55,41.49,'N'],'height':-307},
#  {'lon':[35,22,29.60,'E'], 'lat':[30,55,42.09,'N'],'height':-308},
#  {'lon':[35,22,30.21,'E'], 'lat':[30,55,42.58,'N'],'height':-308},
#  {'lon':[35,22,30.72,'E'], 'lat':[30,55,43.04,'N'],'height':-303},
#  {'lon':[35,22,32.02,'E'], 'lat':[30,55,43.25,'N'],'height':-303},
#  {'lon':[35,22,33.74,'E'], 'lat':[30,55,43.52,'N'],'height':-308},
#  {'lon':[35,22,34.97,'E'], 'lat':[30,55,43.69,'N'],'height':-305}]


spline_points_car1, spline_points_der_car1,euler_initial_angles_car1 = Utils.generate_spline_path(spline_position_dms, cesium_transformation, spline_param = 3, num_samples = 500, add_z = 0)


spline_position_dms_car2 = [
 {'lon':[35,22,35.80,'E'], 'lat':[30,55,42.44,'N'],'height':-303},
 {'lon':[35,22,34.32,'E'], 'lat':[30,55,42.23,'N'],'height':-303},
 {'lon':[35,22,32.67,'E'], 'lat':[30,55,42.04,'N'],'height':-303},
 {'lon':[35,22,31.21,'E'], 'lat':[30,55,41.97,'N'],'height':-303},
 {'lon':[35,22,29.92,'E'], 'lat':[30,55,41.70,'N'],'height':-303}]

spline_points_car2, spline_points_der_car2,euler_initial_angles_car2 = Utils.generate_spline_path(spline_position_dms_car2, cesium_transformation, spline_param = 3, num_samples = 500, add_z = 0)


print("initilizing radar")
rcs_file_path = '/home/ronim/Documents/radar_sim/radar_rcs_maps/rcs_ford_raptor_1.pkl'
radar_prop_path = '/home/ronim/Documents/radar_sim/radar_parameters/MAGOS.yaml'
text_for_image = {}
delta_az = 80
delta_el = 50
radar_angle = [0, 90, 180]               
origin_world_radar = np.array([transformed_vertices[0], transformed_vertices[1], transformed_vertices[2]+3])
radar = Radar(rcs_file_path, radar_prop_path, "ball", origin_world_radar, radar_angle, delta_az=delta_az, delta_el=delta_el)



world = World()
world.reset()

enviorment = Enviorment(world, light_path="/World/sky/DomeLight", floor_path="/World/z_upv2", texture_sky = '/home/ronim/Downloads/sly_chat.png', light_intensity = 1000)
enviorment.set_dome_direction({'Y':180, 'Z':180})


world.get_physics_context().set_gravity(-500.0)




DynamicCuboid(prim_path="/World/cube", color=np.array([0, 255, 0]))
cube = Asset("/World/cube")
cube.set_pose(translation=np.array([transformed_vertices[0], transformed_vertices[1], transformed_vertices[2]+3]), orientation = np.array([0,-30,-100]))
camera = CameraClass(prim_path = "/World/cube/camera",orientation = np.array([0, 0, 0]),translation = [0,0,0.0],resolution = (width, height))
camera.camera.initialize()


stage = omni.usd.get_context().get_stage()




# cube2 = Asset(car_prim_path)
car1 = Asset(car1_path, usd_load_path=car_asset_path, rigid_prim=True, scale=[scale_axis[car1_path][0]*2]*3)
tree = Asset(tree_path, usd_load_path=tree_asset_path, rigid_prim=False, scale=[scale_axis[tree_path][0]]*3)
car2 = Asset(car2_path, usd_load_path=van_asset_path, rigid_prim=True, scale=[scale_axis[car2_path][0]*2]*3)

controller = Controller(imapper.cfg)
cube.disable_gravity()




prim = car1.get_prim()
mass_kg = 1000
mapi = UsdPhysics.MassAPI.Apply(prim)
mapi.GetMassAttr().Set(float(mass_kg))


prim = car2.get_prim()
mass_kg = 1000
mapi = UsdPhysics.MassAPI.Apply(prim)
mapi.GetMassAttr().Set(float(mass_kg))




frame_count = 0
passed_time = 0
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()




car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))
car2.set_pose(translation=np.array(spline_points_car2[0]), orientation = np.array([0,0,180-euler_initial_angles_car2]))



angular_velocity = 0
show_pd_real_target = False

from PolarPlotReusable import PolarPlotReusable

polar_plot = PolarPlotReusable(size=(220,220), r_max=400)

while simulation_app.is_running():
    if world.is_playing():
        now = time.monotonic()
        # ------------------ physics update ------------------

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
        translation, orientation = car1.get_position_and_orientation()
        current_heading_vector = car1.get_local_orientation(orientation)

        angular_velocity_car1, linear_velocity_car1 = controller.get_velocity_parameters_spline_path(translation,current_heading_vector,spline_points_car1,spline_points_der_car1,'car1', kp = 200)

        # Set linear velocity to follow the tangent, and angular velocity to turn
        car1.set_velocity_world(linear_velocity_car1)
        car1.set_angular_velocity_world(np.array([0, 0, angular_velocity_car1])) # Use world frame for simplicity

        # Get the final velocity for your radar
        velocity = car1.get_linear_velocity(orientation)
    
        if controller.reset_asset(mapping, 'car1'):
            car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))
            car2.set_pose(translation=np.array(spline_points_car2[0]), orientation = np.array([0,0,180-euler_initial_angles_car2]))

        #___________________________________________________________________________________________________________

        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        translation_car2, orientation_car2 = car2.get_position_and_orientation()
        current_heading_vector_car2 = car2.get_local_orientation(orientation_car2)

        angular_velocity_car2, linear_velocity_car2 = controller.get_velocity_parameters_spline_path(translation_car2,current_heading_vector_car2,spline_points_car2,spline_points_der_car2,'car2', kp = 50)

        # Set linear velocity to follow the tangent, and angular velocity to turn
        car2.set_velocity_world(linear_velocity_car2)
        car2.set_angular_velocity_world(np.array([0, 0, angular_velocity_car2])) # Use world frame for simplicity

        # Get the final velocity for your radar
        velocity_car2 = car2.get_linear_velocity(orientation_car2)
        #___________________________________________________________________________________________________________


        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True
        target, false_alarm = radar.get_detections(translation, orientation, velocity)
        target2, false_alarm2 = radar.get_detections(translation_car2, orientation_car2, velocity_car2)
        target = radar.check_if_targets_in_same_bin( [target, target2], 'range')

        target = {k: np.concatenate([np.atleast_1d(d.get(k, [])) for d in target]) for k in set().union(*target)}

        detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])) for key in target.keys()}

        # false_alarm =[]
        radar_rb.push(detection) # update the radar detection ring buffer 

        # if should_render is True, get the camera frame and add it to the queue for processing
        if should_render:
            passed_time += now - last_time
            last_time = now
            if print_detection:
                all_data_text = radar.print_detections(text_for_image,target, false_alarm, passed_time)
                if show_pd_real_target:
                    all_data_text = f"{all_data_text} \n real target{round(radar.target_range,2)} \n pd {round(radar.radar_properties['pd'],2)}"
            frame_rgb_bytes = camera.frame_in_bytes(None, az_deg=detection.get('azimuth', None), r_m=detection.get('range', None), polar_plot = polar_plot)
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing

    frame_count += 1
