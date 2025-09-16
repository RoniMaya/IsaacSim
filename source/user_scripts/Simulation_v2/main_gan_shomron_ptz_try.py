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

from path_define import (
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,
    RCS_FILE_PATH, RADAR_PROP_PATH
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH)


width = 640
height = 480
physics_frequency = 120  # Hz
rendering_frequency = 30  # Frames per second
detection_frequency = 1  # Hz
frame_count = 0
passed_time = 0
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()
render_every_n_frame = int(physics_frequency / rendering_frequency)


path_mesh = '/home/ronim/omniverse_assets_local/Gan_Shomron/odm_georeferencing/coords.txt'
with open(path_mesh, "r") as f:
    lines = f.read().strip().splitlines()
utm_data = {
    "datum": lines[0].split(' ')[0],
    "projection": lines[0].split(' ')[1],
    "zone_num": int(lines[0].split(' ')[2][:-1]),
    "zone_letter": lines[0].split(' ')[2][-1],
    "lat": float(lines[1].split(' ')[0]),
    "lon": float(lines[1].split(' ')[1])
}

imgr = InputManager()
input_server = InputServer(imgr)
imapper = InputMapper(CFG_FILE)
mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}


geojson_path = '/home/ronim/omniverse_assets_local/gs_coordinates/road.geojson'
geojson_loader = GeoJSONLoader(geojson_path, utm_data)
utm_coords = geojson_loader.latlon_to_utm(geojson_loader.lat, geojson_loader.lon)
road_enu_coords = geojson_loader.utm_to_enu(utm_coords[0], utm_coords[1])
spline_points_car1, spline_points_der_car1,euler_initial_angles_car1 = Utils.generate_spline_path_from_enu(road_enu_coords, spline_param = 3, num_samples = 1000, add_z = 0)


geojson_path = '/home/ronim/omniverse_assets_local/gs_coordinates/villa_gs.geojson'
geojson_loader = GeoJSONLoader(geojson_path, utm_data)
utm_coords = geojson_loader.latlon_to_utm(geojson_loader.lat, geojson_loader.lon)
small_house_enu_coords = geojson_loader.utm_to_enu(utm_coords[0], utm_coords[1])





car1_path = "/World/car1"




radar = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=small_house_enu_coords[0])


# Start background threads -----------------------------------------------------------
# start server thread - this will listen to controller inputs
threading.Thread(target=input_server.start, daemon=True).start()
video_rb = RingBuffer(capacity=8, drop_policy="latest")
VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread


# mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
# 'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}

radar_rb = RingBuffer(capacity=512, drop_policy="latest")
radar_pub = DetectionPublisherMQTT(radar_rb, radar.radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()



# Add env--------------------------------------------------------------------------------------------
scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ car1_path],
                                                                                            [ CAR_ORANGE_ASSET_PATH])}
open_stage(STAGE_PATH_GAN_SHOMRON)



print("initilizing radar")

text_for_image = {}
world = World()


enviorment = Enviorment(world, light_path="/World/sky/DomeLight", floor_path="/World/odm_textured_model_geo", texture_sky = TEXTURE_SKY, light_intensity = 1000)
# enviorment.set_dome_direction({'Y':0, 'Z':180})

curves = UsdGeom.BasisCurves.Define(enviorment.stage, "/World/RadarCenter")
curves.CreateTypeAttr("linear")
curves.CreateCurveVertexCountsAttr([2])
curves.CreatePointsAttr([radar.radar_origin, radar.radar_origin + radar.radar_properties['fwd_radar'] * radar.radar_properties['r_max']])
curves.CreateWidthsAttr([0.2])
UsdGeom.Gprim(curves.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(0.0,1.0,0.0)])  # green
world.reset()


world.get_physics_context().set_gravity(-500.0)


small_house_enu_coords = np.vstack(geojson_loader.get_collisions( world, small_house_enu_coords))
spline_points_car1 = np.vstack(geojson_loader.get_collisions( world, spline_points_car1))
pole_h = 20
# new_z = hit[2][0].hit_position 

# # --- build joint specs with human-friendly degrees ---
# yaw_spec = Joints.make_joint_spec(
#     axis="Z",
#     parent_pos=Gf.Vec3f(0, 0, +0.1),   # top face of base (assuming 1 m cube)
#     child_pos= Gf.Vec3f(0, 0, -0.1),   # bottom face of middle
#     lower_deg=-180, upper_deg=180,
#     stiffness=0.0, damping=300.0,
#     max_force=1000.0,
#     target_vel_deg_s=60.0
# )

# pitch_spec = Joints.make_joint_spec(
#     axis="Y",
#     parent_pos=Gf.Vec3f(0, 0, +0.5),   # top face of middle
#     child_pos= Gf.Vec3f(0, 0, -0.5),   # bottom face of child
#     lower_deg=-45, upper_deg=45,
#     stiffness=0.0, damping=300.0,
#     max_force=1000.0,
#     target_vel_deg_s=60.0
# )




drive_base, drive_middle = ptz.create_ptz_camera(enviorment.stage, prim_path = "/World/camera_prim", 
                         translation = np.array([small_house_enu_coords[0,0], small_house_enu_coords[0,1],small_house_enu_coords[0,2]+pole_h]))
camera = CameraClass(prim_path = "/World/camera_prim/child/camera",orientation = np.array([0, 0, 0]),translation = [0.05,0,0.25],resolution = (width, height))
camera.camera.initialize()




# VisualCuboid(prim_path="/World/camera_prim/base", color=np.array([0, 0, 255]))
# camera_base = Asset("/World/camera_prim/base", rigid_prim = False)
# camera_base.set_pose(translation=np.array([small_house_enu_coords[0,0], small_house_enu_coords[0,1],small_house_enu_coords[0,2]]), orientation = np.array([0,0,0]))




# DynamicCuboid(prim_path="/World/camera_prim/middle", color=np.array([0, 0, 255]))
# camera_middle = Asset("/World/camera_prim/middle", rigid_prim = True)
# camera_middle.set_pose(translation=np.array([small_house_enu_coords[0,0], small_house_enu_coords[0,1],small_house_enu_coords[0,2]+1]), orientation = np.array([0,0,0]))
# camera_middle.disable_gravity()



# DynamicCuboid(prim_path="/World/camera_prim/child", color=np.array([0, 0, 255]))
# camera_child = Asset("/World/camera_prim/child", rigid_prim = True)
# camera_child.set_pose(translation=np.array([small_house_enu_coords[0,0], small_house_enu_coords[0,1],small_house_enu_coords[0,2]+2]), orientation = np.array([0,0,0]))
# camera_child.disable_gravity()


# joint_props = {'parent_position': Gf.Vec3f(0, 0, 1), 'child_position': Gf.Vec3f(0, 0, -1),
#             'lower_limit': -45, 'upper_limit': 45,'max_force': 1000.0, 
#             'target_velocity': 60.0, 'damping': 300.0, 'stiffness': 0.0,
#             'revolution_axis': 'Z'}

# camera_base.define_joint(enviorment.stage, joint_path = "/World/camera_prim/joints/base", connected_prim_path = camera_middle.prim_path, joint_props = joint_props)

# joint_props = {'parent_position': Gf.Vec3f(0, 0.5, 1), 'child_position': Gf.Vec3f(0, 0.5, -1),
#             'lower_limit': -180, 'upper_limit': 180,'max_force': 1000.0, 'target_velocity': 60.0,
#             'damping': 300.0, 'stiffness': 0.0, 'revolution_axis': 'Y'}

# camera_middle.define_joint(enviorment.stage, joint_path = "/World/camera_prim/joints/middle", connected_prim_path = camera_child.prim_path, joint_props = joint_props)




car1 = Asset(car1_path, usd_load_path=CAR_ORANGE_ASSET_PATH, rigid_prim=True, scale=[scale_axis[car1_path][0]]*3)

controller = Controller(imapper.cfg)
# camera_cube.disable_gravity()




prim = car1.get_prim()
mass_kg = 1000
mapi = UsdPhysics.MassAPI.Apply(prim)
mapi.GetMassAttr().Set(float(mass_kg))







car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))


angular_velocity = 0
show_pd_real_target = False

from PolarPlotReusable import PolarPlotReusable

polar_plot = PolarPlotReusable(size=(220,220), r_max=400)

time_to_wait_physics = 1.0 / physics_frequency
time_to_restart_scenario = 60*2
restart_time = time.monotonic()
while simulation_app.is_running():
    if world.is_playing():
        now = time.monotonic()
        # ------------------ physics update ------------------

        # get pressed keys from the input manager and update the physics and camera
        pressed_keys = imgr.snapshot()
        mapping = imapper.calculate_mapping(pressed_keys)  # {'keyboard1': {'W','A',...}} -> {'throttle': 1.0, 'steering': -1.0, ...}

        # update velocity, orientation for each assets (+zoom for camera)__________________________________________
        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        # velocity = controller.update_velocity_direction(mapping, 'camera')
        # camera_orientation = controller.update_orientation(mapping, 'camera')
        zoom_factor = controller.zoom_factor(mapping, 'camera')
        rot_flag = controller.rot_flag(mapping, 'camera')

        #----------------------------------------------------------------------------
        # set the velocity, orientation and zoom of the "camera" cube
        # translation, orientation = camera_cube.get_position_and_orientation()
        # translation_poll, orientation_poll = camera_poll.get_position_and_orientation()
        # camera_poll.set_angular_velocity_local([np.array([0,0,camera_orientation[2]])], orientation_poll)

        # camera_cube.set_angular_velocity_local([np.array([0,camera_orientation[1],0])], orientation)
        # camera_cube.set_velocity_local(velocity, orientation)
        camera.zoom_camera(zoom_factor)
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
    
        if controller.reset_asset(mapping, 'car1') or now - restart_time  >= time_to_restart_scenario :
            restart_time = now
            car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))

        #___________________________________________________________________________________________________________


        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True
        target, false_alarm = radar.get_detections(translation, orientation, velocity, target_id = "car1")
        target = radar.check_if_targets_in_same_bin( [target], 'range')

                # Spin at 30 deg/s (positive around the joint axis)
        drive_base.GetTargetPositionAttr().Set(0.0)   # ignore position
        drive_base.GetTargetVelocityAttr().Set(rot_flag[2] * 60.0)  # deg/s
        drive_middle.GetTargetPositionAttr().Set(0.0)   # ignore position
        drive_middle.GetTargetVelocityAttr().Set(rot_flag[1] * 60.0)  # deg/s


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
            frame_rgb_bytes = camera.frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = polar_plot)
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
        time.sleep(time_to_wait_physics)
    frame_count += 1
