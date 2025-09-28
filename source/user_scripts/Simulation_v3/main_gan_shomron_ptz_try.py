from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({'headless': True})

from InputUtils.InputManager import InputManager
from InputUtils.InputServer import InputServer
from InputUtils.InputMapper import InputMapper
import threading


import numpy as np,scipy

from isaacsim.core.utils import rotations
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicCylinder
import numpy as np

from Asset import Asset
from CameraClass import CameraClass
from Enviorment import Enviorment
from Controller import Controller
from Publishers.RingBuffer import RingBuffer
from Publishers.VideoPublisher import VideoPublisher
import Utils
import os
import time
from Radar import Radar
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdLux,PhysxSchema, Sdf, Vt

import omni.usd
from scipy.interpolate import splprep, splev
from Publishers.DetectionPublisherMQTT import DetectionPublisherMQTT
import utm
# from Publishers.DetectionPublisher import DetectionPublisher

import paho.mqtt.client as mqtt
import cv2
from GeoJSONLoader import GeoJSONLoader
from isaacsim.core.utils import rotations 
import Joints
from DynamicAssets import ptz
from PolarPlotReusable import PolarPlotReusable

from path_define import (
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, GEOJSON_GS
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, GEOJSON_GS)


width = 1280//2
height = 720//2
physics_frequency = 120  # Hz
rendering_frequency = 30  # Frames per second
detection_frequency = 1  # Hz
frame_count = 0
passed_time = 0
time_to_restart_scenario = 60*2
camera_height = 5
stage_path = STAGE_PATH_GAN_SHOMRON
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()
render_every_n_frame = int(physics_frequency / rendering_frequency)
text_for_image = {}
show_pd_real_target = False
polar_plot = PolarPlotReusable(size=(220,220), r_max=400)
time_to_wait_physics = 1.0 / physics_frequency

car1_path = "/World/vehicles/car1"
camera1_path = "/World/camera1_prim"
camera2_path = "/World/camera2_prim"

radar_path = "/World/radar"
radar2_path = "/World/radar2"

imgr = InputManager()
input_server = InputServer(imgr)
imapper = InputMapper(CFG_FILE)
controller = Controller(imapper.cfg)

mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}



utm_data = Utils.open_coords_file(COORDINATES_GS)
geojson_loader = GeoJSONLoader(utm_data, 'road',GEOJSON_GS)
spline_points_car1, spline_points_der_car1,euler_initial_angles_car1 = geojson_loader.get_spline()

geojson_loader = GeoJSONLoader(utm_data, 'camera2',GEOJSON_GS)
camera2_location = geojson_loader.lat_lon_to_enu()


geojson_loader = GeoJSONLoader(utm_data, 'camera1',GEOJSON_GS)
camera1_location = geojson_loader.lat_lon_to_enu()


radar = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=camera1_location[0])
radar2 = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=camera2_location[0])


print("initilizing radar")

# Start background threads -----------------------------------------------------------
# start server thread - this will listen to controller inputs
threading.Thread(target=input_server.start, daemon=True).start()
video_rb = RingBuffer(capacity=8, drop_policy="latest")
VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread

radar_rb = RingBuffer(capacity=512, drop_policy="latest")
radar_pub = DetectionPublisherMQTT(radar_rb, radar.radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()

# Add env--------------------------------------------------------------------------------------------
scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ car1_path],                                                                                            [ CAR_ORANGE_ASSET_PATH])}
# open_stage(stage_path)
enviorment = Enviorment(stage_path="/World", light_path="/World/sky/DomeLight", floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)
enviorment.load_prim_with_collision(f'{STAGE_PATH_GAN_SHOMRON}', "/World/odm_textured_model_geo")
world = World()


radar.plot_radar_direction( enviorment.stage, prim_path=f"{radar_path}/radar_direction")
radar2.plot_radar_direction( enviorment.stage, prim_path=f"{radar2_path}/radar2_direction")
enviorment.set_gravity(physics_path = f"/World/Physics")
world.reset()


# get Z value for the assets (collision with the ground)
camera1_location = np.vstack(geojson_loader.get_collisions( world, camera1_location))
spline_points_car1 = np.vstack(geojson_loader.get_collisions( world, spline_points_car1))
camera2_location = np.vstack(geojson_loader.get_collisions( world, camera2_location))

# define camera
drive_base, drive_middle = ptz.create_ptz_camera(enviorment.stage, prim_path = camera1_path, 
                         translation = np.array([camera1_location[0,0], camera1_location[0,1],camera1_location[0,2]+camera_height]))
camera = CameraClass(prim_path = f"{camera1_path}/child/camera",orientation = np.array([0, 0, 0]),translation = [0.05,0,0.25],resolution = (width, height))
camera.camera.initialize()


# define camera
drive_base_cam2, drive_middle_cam2 = ptz.create_ptz_camera(enviorment.stage, prim_path = camera2_path, 
                         translation = np.array([camera2_location[0,0], camera2_location[0,1],camera2_location[0,2]+camera_height]))
camera2 = CameraClass(prim_path = f"{camera2_path}/child/camera",orientation = np.array([0, 0, 180]),translation = [0.05,0,0.25],resolution = (width, height))
camera2.camera.initialize()




# define car1
car1 = Asset(car1_path, usd_load_path=CAR_ORANGE_ASSET_PATH, rigid_prim=True, scale=[scale_axis[car1_path][0]]*3)
car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]), local = False)



asset = 'cam1'
restart_time = time.monotonic()
dt = 1.0 / physics_frequency
old_orientation = np.array([0,0,0])
while simulation_app.is_running():
    if world.is_playing():
        now = time.monotonic()
        # ------------------ physics update ------------------

        # get pressed keys from the input manager and update the physics and camera
        pressed_keys = imgr.snapshot()
        print(pressed_keys)
        mapping = imapper.calculate_mapping(pressed_keys)  # {'keyboard1': {'W','A',...}} -> {'throttle': 1.0, 'steering': -1.0, ...}

        # update velocity, orientation for each assets (+zoom for camera)__________________________________________
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        # velocity = controller.update_velocity_direction(mapping, 'camera')
        camera_orientation = controller.update_orientation(mapping, asset)
        camera_orientation = old_orientation+camera_orientation*dt
        old_orientation = camera_orientation.copy()

        asset = imapper.active_camera_target

        zoom_factor = controller.zoom_factor(mapping, asset)
        rot_flag = controller.rot_flag(mapping, asset)
        print(mapping)

        #----------------------------------------------------------------------------
        # set the velocity, orientation and zoom of the "camera" cube
        # translation, orientation = camera_cube.get_position_and_orientation()
        # translation_poll, orientation_poll = camera_poll.get_position_and_orientation()
        # camera_poll.set_angular_velocity_local([np.array([0,0,camera_orientation[2]])], orientation_poll)

        # camera_cube.set_angular_velocity_local([np.array([0,camera_orientation[1],0])], orientation)
        # camera_cube.set_velocity_local(velocity, orientation)
        #-------------------------------------------------------------------------
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        translation, orientation = car1.get_position_and_orientation()
        current_heading_vector = car1.get_local_orientation(orientation)

        angular_velocity_car1, linear_velocity_car1 = controller.get_velocity_parameters_spline_path(translation,current_heading_vector,spline_points_car1,spline_points_der_car1,'car1', kp = 200)

        # Set linear velocity to follow the tangent, and angular velocity to turn
        car1.set_velocity(linear_velocity_car1, local = False)
        car1.set_angular_velocity(np.array([0, 0, angular_velocity_car1]), local = False) # Use world frame for simplicity

        # Get the final velocity for your radar
        velocity = car1.get_linear_velocity(orientation)
    
        if controller.reset_asset(mapping, asset) or now - restart_time  >= time_to_restart_scenario :
            restart_time = now
            car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]), local = False )

        #___________________________________________________________________________________________________________


        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True

                # Spin at 30 deg/s (positive around the joint axis)
        if asset == 'cam1':
            camera.transform_camera(camera_orientation,[0,0,0])
            drive_base.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_base.GetTargetVelocityAttr().Set(0 * 50.0)  # deg/s
            drive_middle.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_middle.GetTargetVelocityAttr().Set(0 * 50.0)  # deg/s
            camera.zoom_camera(zoom_factor)
            target, false_alarm = radar.get_detections(translation, orientation, velocity, target_id = "car1")

        if asset == 'cam2':
            drive_base_cam2.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_base_cam2.GetTargetVelocityAttr().Set(rot_flag[2] * 50.0)  # deg/s
            drive_middle_cam2.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_middle_cam2.GetTargetVelocityAttr().Set(rot_flag[1] * 50.0)  # deg/s
            camera2.zoom_camera(zoom_factor)
            target, false_alarm = radar2.get_detections(translation, orientation, velocity, target_id = "car1")




        target = radar.check_if_targets_in_same_bin( [target], 'range')
        target = {k: np.concatenate([np.atleast_1d(d.get(k, [])).tolist() for d in target]) for k in set().union(*target)}
        detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])).tolist() for key in target.keys()}

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
            if asset == 'cam2':
                frame_rgb_bytes = camera2.frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = polar_plot)

            else:
                frame_rgb_bytes = camera.frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = polar_plot)
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
        time.sleep(time_to_wait_physics)
    frame_count += 1
