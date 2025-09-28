from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

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
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON,STAGE_PATH_TEL_KUDNA,STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,WHITE_TOYOTA,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS,COORDINATES_TK,COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3, GEOJSON_GS, GEOJSON_TK, GEOJSON_LATRUN
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, STAGE_PATH_TEL_KUDNA, STAGE_PATH_LATRUN_1, STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, COORDINATES_TK, GEOJSON_GS, GEOJSON_TK)


width = 640
height = 480
physics_frequency = 60  # Hz
rendering_frequency = 30  # Frames per second
detection_frequency = 1  # Hz
frame_count = 0
passed_time = 0
time_to_restart_scenario = 60*2
camera_height = 20
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()
render_every_n_frame = int(physics_frequency / rendering_frequency)
text_for_image = {}
show_pd_real_target = False
polar_plot = PolarPlotReusable(size=(220,220), r_max=400)
time_to_wait_physics = 1.0 / physics_frequency
coords_path = COORDINATES_LATRUN_1
stage_path = STAGE_PATH_LATRUN_1

drone_path = "/World/drone"
camera_path = "/World/camera_prim"
camera2_path = "/World/camera2_prim"
radar_path = "/World/radar"
toyota_path  = "/World/vehicles/toyota"

imgr = InputManager()
input_server = InputServer(imgr)
imapper = InputMapper(CFG_FILE)
controller = Controller(imapper.cfg)

mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}



utm_data = Utils.open_coords_file(coords_path)
geojson_loader = GeoJSONLoader(utm_data, 'drone_path',GEOJSON_LATRUN)
spline_points_drone, spline_points_der_drone,euler_initial_angles_drone = geojson_loader.get_spline()
spline_points_drone[:,2] = spline_points_drone[:,2]*0 + 500




utm_data = Utils.open_coords_file(coords_path)
geojson_loader = GeoJSONLoader(utm_data, 'road_2',GEOJSON_LATRUN)
spline_points_toyota, spline_points_der_toyota,euler_initial_angles_toyota = geojson_loader.get_spline()


utm_data_2 = Utils.open_coords_file(COORDINATES_LATRUN_2)
geojson_loader2 = GeoJSONLoader(utm_data, 'camera_test',GEOJSON_LATRUN)
camera2_location = geojson_loader2.lat_lon_to_enu()


primiter = GeoJSONLoader(utm_data, 'perimiter',GEOJSON_LATRUN)
spline_points_primiter, spline_points_der_primiter,euler_initial_angles_primiter = primiter.get_spline()
spline_points_primiter[:,2] = spline_points_primiter[:,2]*0 + 20



utm_data_3 = Utils.open_coords_file(COORDINATES_LATRUN_3)

radar = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=camera2_location[0])

print("initilizing radar")

# Start background threads -----------------------------------------------------------
# start server thread - this will listen to controller inputs
threading.Thread(target=input_server.start, daemon=True).start()
video_rb = RingBuffer(capacity=8, drop_policy="latest")
VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread

radar_rb = RingBuffer(capacity=512, drop_policy="latest")
radar_pub = DetectionPublisherMQTT(radar_rb, radar.radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()

scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ toyota_path],[ WHITE_TOYOTA])}



enviorment = Enviorment(stage_path='/World', light_path="/World/sky/DomeLight",  floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)



def record_to_EN(rec):
    # Your records are already UTM (meters), just badly named.
    E = float(rec['lat'])  # actually Easting
    N = float(rec['lon'])  # actually Northing
    return E, N
e1, n1 = record_to_EN(utm_data)
e2, n2 = record_to_EN(utm_data_2)
e3, n3 = record_to_EN(utm_data_3)

new_origin_1  = [0.0, 0.0, 0.0]
new_origin_21 = [e2 - e1, n2 - n1, 0.0]
new_origin_31 = [e3 - e1, n3 - n1, 0.0]


enviorment.load_prim_with_collision(f'{STAGE_PATH_LATRUN_1}', "/World/Terrain/FirstTerrain")
enviorment.load_prim_with_collision(f'{STAGE_PATH_LATRUN_2}', "/World/Terrain/SecondTerrain")
enviorment.load_prim_with_collision(f'{STAGE_PATH_LATRUN_3}', "/World/Terrain/ThirdTerrain")


enviorment.translate_terrain("/World/Terrain/FirstTerrain",new_origin_1)
enviorment.translate_terrain("/World/Terrain/SecondTerrain",new_origin_21)
enviorment.translate_terrain("/World/Terrain/ThirdTerrain",new_origin_31)


enviorment.set_gravity(physics_path = "/World/Physics")


world = World()

# get Z value for the assets (collision with the ground)
camera2_location = np.vstack(geojson_loader.get_collisions( world, camera2_location))
spline_points_toyota = np.vstack(geojson_loader.get_collisions( world, spline_points_toyota))
spline_points_toyota[0,2] = spline_points_toyota[0,2] + 0.5
# spline_points_car1 = np.vstack(geojson_loader.get_collisions( world, spline_points_car1))
# define car1
DynamicCuboid(prim_path=f"{drone_path}", color=np.array([0, 255, 0]))
drone = Asset(drone_path, rigid_prim=True)

camera = CameraClass(prim_path = f"{drone.prim_path}/sensors/camera",orientation = np.array([0, 90, 0]),translation = [0,0,-1],resolution = (width, height))
toyota = Asset(toyota_path, usd_load_path=WHITE_TOYOTA, rigid_prim=True, scale=[scale_axis[toyota_path][0]*200]*3)

drive_base_cam2, drive_middle_cam2 = ptz.create_ptz_camera(enviorment.stage, prim_path = camera2_path, 
                         translation = np.array([camera2_location[0,0], camera2_location[0,1],camera2_location[0,2]+camera_height]))
camera2 = CameraClass(prim_path = f"{camera2_path}/child/camera",orientation = np.array([0, 0, 180]),translation = [0.05,0,0.25],resolution = (width, height))

# Create the curve prim

curve = UsdGeom.BasisCurves.Define(enviorment.stage, "/World/BaseBorder")
curve.CreatePointsAttr([Gf.Vec3f(*c) for c in spline_points_primiter])
curve.CreateCurveVertexCountsAttr([len(spline_points_primiter)])
curve.CreateTypeAttr("linear")  # straight segments
curve.CreateWidthsAttr([10.0])   # line thickness in viewport
# Assign per-vertex colors (interpolated between points)
colors = [
    (1.0, 0.0, 0.0),  # red
    (0.0, 1.0, 0.0),  # green
    (0.0, 0.0, 1.0),  # blue
    (1.0, 1.0, 0.0),  # yellow
    (1.0, 1.0, 0.0),  # yellow
    (1.0, 0.0, 0.0)   # back to red
]
curve.CreateDisplayColorAttr([Gf.Vec3f(*c) for c in colors])



world.reset()


drone.set_pose(translation=np.array(spline_points_drone[0]), orientation = np.array([0,0,euler_initial_angles_drone]))
drone.disable_gravity()

toyota.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,euler_initial_angles_toyota]),local = False)
camera.camera.initialize()
camera2.camera.initialize()

# define camera




asset = 'cam1'
dt = 1.0 / physics_frequency
old_orientation = np.array([0,90,0])
world.step(render=False)






while simulation_app.is_running():
    if world.is_playing():
        now = time.monotonic()
        # ------------------ physics update ------------------

        # get pressed keys from the input manager and update the physics and camera
        pressed_keys = imgr.snapshot()
        mapping = imapper.calculate_mapping(pressed_keys)  # {'keyboard1': {'W','A',...}} -> {'throttle': 1.0, 'steering': -1.0, ...}
        # update velocity, orientation for each assets (+zoom for camera)__________________________________________
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        velocity_control = controller.update_velocity_direction(mapping, asset)
        # zoom_factor = controller.zoom_factor(mapping, asset)

        camera_orientation = controller.update_orientation(mapping, asset)
        camera_orientation = old_orientation+camera_orientation*dt
        old_orientation = camera_orientation.copy()


        asset = imapper.active_camera_target

        zoom_factor = controller.zoom_factor(mapping, asset)
        rot_flag = controller.rot_flag(mapping, asset)

        #----------------------------------------------------------------------------
        # set the velocity, orientation and zoom of the "camera" cube
        # translation, orientation = camera_cube.get_position_and_orientation()

        # camera_cube.set_angular_velocity([np.array(camera_orientation)], orientation)
        # camera_cube.set_velocity(velocity, orientation)
        camera.zoom_camera(zoom_factor)
        #-------------------------------------------------------------------------
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        translation_drone, orientation_drone = drone.get_position_and_orientation()
        current_heading_vector = drone.get_local_orientation(orientation_drone)
        angular_velocity_drone, linear_velocity_drone = controller.get_velocity_parameters_spline_path(translation_drone,current_heading_vector,spline_points_drone,spline_points_der_drone,'drone', kp = 200)

        # Set linear velocity to follow the tangent, and angular velocity to turn
        linear_velocity_drone = velocity_control if np.linalg.norm(velocity_control) > 0 else linear_velocity_drone
        drone.set_velocity(linear_velocity_drone, local = False)
        # car1.set_angular_velocity(np.array([0, 0, angular_velocity_car1]), local = False) # Use world frame for simplicity


        translation_toyota, orientation_toyota = toyota.get_position_and_orientation()
        current_heading_vector = toyota.get_local_orientation(orientation_toyota)

        angular_velocity_toyota, linear_velocity_toyota = controller.get_velocity_parameters_spline_path(translation_toyota,current_heading_vector,spline_points_toyota,spline_points_der_toyota,'toyota', kp = 200)
        # Set linear velocity to follow the tangent, and angular velocity to turn
        toyota.set_velocity(linear_velocity_toyota, local = False)
        toyota.set_angular_velocity(np.array([0, 0, angular_velocity_toyota]), local = False) # Use world frame for simplicity



        # Get the final velocity for your radar
        velocity = drone.get_linear_velocity(orientation_drone)
        if controller.reset_asset(mapping, asset) :
            restart_time = now
            toyota.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,euler_initial_angles_toyota]),local = False)


        #___________________________________________________________________________________________________________


        if asset == 'cam2':
            drive_base_cam2.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_base_cam2.GetTargetVelocityAttr().Set(rot_flag[2] * 50.0)  # deg/s
            drive_middle_cam2.GetTargetPositionAttr().Set(0.0)   # ignore position
            drive_middle_cam2.GetTargetVelocityAttr().Set(rot_flag[1] * 50.0)  # deg/s
            camera2.zoom_camera(zoom_factor)



        camera.transform_camera(camera_orientation,[0,0,0])
        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

        logic_time = time.perf_counter()

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True
        target, false_alarm = radar.get_detections(translation_toyota, orientation_toyota, velocity, target_id = "car1")
        target = radar.check_if_targets_in_same_bin( [target], 'range')

        step_time = time.perf_counter()



        target = {k: np.concatenate([np.atleast_1d(d.get(k, [])).tolist() for d in target]) for k in set().union(*target)}

        detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])).tolist() for key in target.keys()}

        false_alarm =[]
        radar_rb.push(detection) # update the radar detection ring buffer 

        # if should_render is True, get the camera frame and add it to the queue for processing

        if should_render:
            passed_time += now - last_time
            last_time = now
            if print_detection:
                # all_data_text = radar.print_detections(text_for_image,target, false_alarm, passed_time)
                if show_pd_real_target:
                    all_data_text = f"{all_data_text} \n real target{round(radar.target_range,2)} \n pd {round(radar.radar_properties['pd'],2)}"
            if asset == 'cam2':
                frame_rgb_bytes = camera2.frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = polar_plot)

            else:
                frame_rgb_bytes = camera.frame_in_bytes(None, az_deg=None, r_m=None, polar_plot = polar_plot)
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
            render_time = time.perf_counter()
            print(f"Logic: {(logic_time - now)*1000:.2f}ms | Step: {(step_time - logic_time)*1000:.2f}ms | Render: {(render_time - step_time)*1000:.2f}ms")

        
        
        if now - last_time < time_to_wait_physics:
            time.sleep(time_to_wait_physics - (now - last_time))
    frame_count += 1
