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
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON,STAGE_PATH_TEL_KUDNA,STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS,COORDINATES_TK,COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3, GEOJSON_GS, GEOJSON_TK, GEOJSON_LATRUN
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, STAGE_PATH_TEL_KUDNA, STAGE_PATH_LATRUN_1, STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, COORDINATES_TK, GEOJSON_GS, GEOJSON_TK)


width = 640
height = 480
physics_frequency = 120  # Hz
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

car1_path = "/World/vehicles/car1"
camera_path = "/World/camera_prim"
camera2_path = "/World/camera2_prim"
radar_path = "/World/radar"

imgr = InputManager()
input_server = InputServer(imgr)
imapper = InputMapper(CFG_FILE)
controller = Controller(imapper.cfg)

mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}



utm_data = Utils.open_coords_file(coords_path)
geojson_loader = GeoJSONLoader(utm_data, 'road_test',GEOJSON_LATRUN)
spline_points_car1, spline_points_der_car1,euler_initial_angles_car1 = geojson_loader.get_spline()


utm_data_2 = Utils.open_coords_file(COORDINATES_LATRUN_2)
geojson_loader2 = GeoJSONLoader(utm_data, 'camera_3',GEOJSON_LATRUN)
camera2_location = geojson_loader2.lat_lon_to_enu()

utm_data_3 = Utils.open_coords_file(COORDINATES_LATRUN_3)
# geojson_loader3 = GeoJSONLoader(utm_data_3, 'camera_3',GEOJSON_LATRUN)
# camera2_location = geojson_loader3.lat_lon_to_enu()


# utm_data_3 = Utils.open_coords_file(COORDINATES_LATRUN_3)
# geojson_loader3 = GeoJSONLoader(utm_data, 'camera_2',GEOJSON_LATRUN)


# new_origin_21 = [12,-322,0]
# new_origin_31 = [28,-660,15]

# camera2_location = camera2_location[0][0:3] + new_origin_21

# camera2_location = [[camera2_location[0],camera2_location[1],camera2_location[2],1]]

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

# Add env--------------------------------------------------------------------------------------------
scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ car1_path],                                                                                            [ CAR_ORANGE_ASSET_PATH])}


# open_stage(stage_path)
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
world_xf = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
stage.SetDefaultPrim(world_xf.GetPrim())
UsdGeom.SetStageMetersPerUnit(stage, 1.0)             # meters
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)       # Z-up
phys = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/Physics"))
phys.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
phys.CreateGravityMagnitudeAttr(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(phys.GetPrim())



def load_prim_with_collision(terrain_path_to_load, terrain_path):
    terrain_xf = UsdGeom.Xform.Define(stage, Sdf.Path(terrain_path))
    terrain_prim = terrain_xf.GetPrim()
    terrain_prim.GetReferences().AddReference(terrain_path_to_load)
    UsdPhysics.CollisionAPI.Apply(terrain_prim)


def translate_terrain(terrain_path,new_origin):

    prim = stage.GetPrimAtPath(terrain_path)
    xf   = UsdGeom.Xformable(prim)
    xf.SetResetXformStack(True)
    xf.ClearXformOpOrder()
    t = xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)  # xformOp:translate
    t.Set(Gf.Vec3d(float(new_origin[0]), float(new_origin[1]), float(new_origin[2])))
    pos = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    print("WORLD translate:", pos)


def offset_from(base, target, z_bias=0.0):
    dE = target['E'] - base['E']   # X
    dN = target['N'] - base['N']   # Y
    dZ = 0.0 + z_bias              # keep flat unless you measured a bias
    return [dE, dN, dZ]

# --- 2) Compute patch offsets *once* (relative to patch 1) ---

utm1 = {'E': utm_data['lat'], 'N': utm_data['lon']}
utm2 = {'E': utm_data_2['lat'], 'N': utm_data_2['lon']}
utm3 = {'E': utm_data_3['lat'], 'N': utm_data_3['lon']}


new_origin_1  = [0.0, 0.0, 0.0]
new_origin_21 = offset_from(utm1, utm2)         # SecondTerrain relative to FirstTerrain
new_origin_31 = offset_from(utm1, utm3)         # ThirdTerrain relative to FirstTerrain



load_prim_with_collision(f'{STAGE_PATH_LATRUN_1}', "/World/Terrain/FirstTerrain")
load_prim_with_collision(f'{STAGE_PATH_LATRUN_2}', "/World/Terrain/SecondTerrain")
load_prim_with_collision(f'{STAGE_PATH_LATRUN_3}', "/World/Terrain/ThirdTerrain")


# new_origin_21 = [utm_data['lat'] - utm_data_2['lat'], -utm_data['lon'] + utm_data_2['lon'], 0]

# new_origin_1 = [0,0, 3]
translate_terrain("/World/Terrain/FirstTerrain",new_origin_1)

# new_origin_21 = [utm_data['lat'] - utm_data_2['lat'], -utm_data['lon'] + utm_data_2['lon'], 0]
translate_terrain("/World/Terrain/SecondTerrain",new_origin_21)



# new_origin_31 = [-utm_data['lat'] + utm_data_3['lat'], -utm_data['lon'] + utm_data_3['lon'], -13]
# new_origin_31 = [40,-660,15]
# new_origin_31 = [28,-677,20]
# new_origin_31 = [54,-660,15]


translate_terrain("/World/Terrain/ThirdTerrain",new_origin_31)


# new_terrain_path = Sdf.Path("/World/Terrain/SecondTerrain")
# second_terrain_xf = UsdGeom.Xform.Define(stage, new_terrain_path)
# second_terrain_prim = second_terrain_xf.GetPrim()
# second_terrain_prim.GetReferences().AddReference(f'{STAGE_PATH_LATRUN_2}')
# UsdPhysics.CollisionAPI.Apply(second_terrain_prim)




# new_terrain_path = Sdf.Path("/World/Terrain/ThirdTerrain")
# third_terrain_xf = UsdGeom.Xform.Define(stage, new_terrain_path)
# third_terrain_prim = third_terrain_xf.GetPrim()
# third_terrain_prim.GetReferences().AddReference(f'{STAGE_PATH_LATRUN_3}')
# UsdPhysics.CollisionAPI.Apply(third_terrain_prim)


world = World()


enviorment = Enviorment(world, light_path="/World/sky/DomeLight", floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)
radar.plot_radar_direction( enviorment.stage, prim_path=f"{radar_path}/radar_direction")
# enviorment.set_gravity()

world.reset()



# new_origin = [-geojson_loader.utm_data['lat'] + geojson_loader2.utm_data['lat'], -geojson_loader.utm_data['lon'] + geojson_loader2.utm_data['lon'], 0]
# # After you define and reference /World/Terrain/SecondTerrain

# prim = stage.GetPrimAtPath("/World/Terrain/SecondTerrain")
# xf   = UsdGeom.Xformable(prim)
# xf.SetResetXformStack(True)
# xf.ClearXformOpOrder()
# t = xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)  # xformOp:translate
# # 3) Set values (orient=identity, scale=1 to preserve shape)
# t.Set(Gf.Vec3d(float(new_origin[0]), float(new_origin[1]), float(new_origin[2])))
# # 4) (Optional) Verify it really moved
# pos = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
# print("WORLD translate:", pos)


# new_origin = [geojson_loader3.utm_data['lat'] - geojson_loader2.utm_data['lat'] + new_origin[0], -geojson_loader3.utm_data['lon'] + geojson_loader2.utm_data['lon'], 0]
# prim = stage.GetPrimAtPath("/World/Terrain/ThirdTerrain")
# xf   = UsdGeom.Xformable(prim)
# xf.SetResetXformStack(True)
# xf.ClearXformOpOrder()
# t = xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)  # xformOp:translate
# # 3) Set values (orient=identity, scale=1 to preserve shape)
# t.Set(Gf.Vec3d(float(new_origin[0]), float(new_origin[1]), float(new_origin[2])))
# # 4) (Optional) Verify it really moved
# pos = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
# print("WORLD translate:", pos)



# get Z value for the assets (collision with the ground)
camera2_location = np.vstack(geojson_loader.get_collisions( world, camera2_location))
spline_points_car1[:,2] = spline_points_car1[:,2]*0 + 500
# spline_points_car1 = np.vstack(geojson_loader.get_collisions( world, spline_points_car1))
# define car1


DynamicCuboid(prim_path=f"{car1_path}", color=np.array([0, 255, 0]))

car1 = Asset(car1_path, rigid_prim=True)
car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))
car1.disable_gravity()



camera = CameraClass(prim_path = f"{car1.prim_path}/sensors/camera",orientation = np.array([0, 90, 0]),translation = [0,0,-1],resolution = (width, height))
camera.camera.initialize()


# define camera
drive_base_cam2, drive_middle_cam2 = ptz.create_ptz_camera(enviorment.stage, prim_path = camera2_path, 
                         translation = np.array([camera2_location[0,0], camera2_location[0,1],camera2_location[0,2]+camera_height]))
camera2 = CameraClass(prim_path = f"{camera2_path}/child/camera",orientation = np.array([0, 0, 180]),translation = [0.05,0,0.25],resolution = (width, height))
camera2.camera.initialize()




asset = 'cam1'
dt = 1.0 / physics_frequency
restart_time = time.monotonic()
old_orientation = np.array([0,90,0])

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
        translation, orientation = car1.get_position_and_orientation()
        current_heading_vector = car1.get_local_orientation(orientation)

        angular_velocity_car1, linear_velocity_car1 = controller.get_velocity_parameters_spline_path(translation,current_heading_vector,spline_points_car1,spline_points_der_car1,'car1', kp = 200)

        # Set linear velocity to follow the tangent, and angular velocity to turn

        linear_velocity_car1 = velocity_control if np.linalg.norm(velocity_control) > 0 else linear_velocity_car1
        car1.set_velocity(linear_velocity_car1, local = False)
        # car1.set_angular_velocity(np.array([0, 0, angular_velocity_car1]), local = False) # Use world frame for simplicity



        # Get the final velocity for your radar
        velocity = car1.get_linear_velocity(orientation)
    
        if controller.reset_asset(mapping, 'car1') or now - restart_time  >= time_to_restart_scenario :
            restart_time = now
            car1.set_pose(translation=np.array(spline_points_car1[0]), orientation = np.array([0,0,euler_initial_angles_car1]))


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

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True
        target, false_alarm = radar.get_detections(translation, orientation, velocity, target_id = "car1")
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
