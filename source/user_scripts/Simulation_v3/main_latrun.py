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
    TEXTURE_SKY, CESIUM_TRANSFORM,WHITE_TOYOTA,TANK,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS,COORDINATES_TK,COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3, GEOJSON_GS, GEOJSON_TK, GEOJSON_LATRUN
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, STAGE_PATH_TEL_KUDNA, STAGE_PATH_LATRUN_1, STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, COORDINATES_TK, GEOJSON_GS, GEOJSON_TK)



def define_ptz_cam(camera_path, camera_height, poi_points, world, enviorment, width, height, camera_name):
    camera_location = np.vstack(geo_json_loader.get_collisions( world, poi_points[camera_name]))
    drive_base_cam, drive_middle_cam = ptz.create_ptz_camera(enviorment.stage, prim_path = camera_path, 
                            translation = np.array([camera_location[0,0], camera_location[0,1],camera_location[0,2]+camera_height]))
    camera = CameraClass(prim_path = f"{camera_path}/child/camera",orientation = np.array([0, 0, 180]),translation = [0.05,0,0.25],resolution = (width, height))
    camera.camera.initialize()
    return drive_base_cam, drive_middle_cam, camera


def generate_asset_in_position(geo_json_loader,asset_path,usd_load_path, world, splines,scale = 2, height=0):
    asset = Asset(asset_path, usd_load_path=usd_load_path, rigid_prim=True, scale=[scale]*3)
    # asset.generate_asset_in_position( geo_json_loader, world, splines, height=height)
    spline_points_toyota = np.vstack(geo_json_loader.get_collisions( world, splines['spline_points']))
    spline_points_toyota[:,2] = spline_points_toyota[:,2] + height
    asset.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,splines['euler_initial_angles']]),local = False)
    return asset


def generate_flying_cube( drone_path,  splines, width, height):
    DynamicCuboid(prim_path=f"{drone_path}", color=np.array([0, 255, 0]))
    drone = Asset(drone_path, rigid_prim=True)
    drone.set_pose(translation=np.array(splines['spline_points'][0]), orientation = np.array([0,0,splines['euler_initial_angles']]),local = False)
    drone.disable_gravity()

    camera = drone.add_camera(width, height)
    return camera, drone

# need to add position in POI

def time_tick_spline_asset(asset,asset_name,splines,controller,velocity_control):
    # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
    translation_asset, orientation_asset = asset.get_position_and_orientation()
    current_heading_vector = asset.get_local_orientation(orientation_asset)
    angular_velocity_asset, linear_velocity_asset = controller.get_velocity_parameters_spline_path(translation_asset,current_heading_vector,splines[asset_name]['spline_points'][:,0:3],splines[asset_name]['spline_points_der'],asset_name, kp = 200)

    # Set linear velocity to follow the tangent, and angular velocity to turn
    linear_velocity_asset = velocity_control if np.linalg.norm(velocity_control) > 0 else linear_velocity_asset
    asset.set_velocity(linear_velocity_asset, local = False)
    asset.set_angular_velocity(np.array([0, 0, angular_velocity_asset]), local = False) # Use world frame for simplicity
    return translation_asset, orientation_asset


if __name__ == "__main__":


    width = 640
    height = 480
    physics_frequency = 60  # Hz
    rendering_frequency = 30  # Frames per second
    detection_frequency = 1  # Hz
    frame_count = 0
    passed_time = 0
    hight_drone = 400
    hight_toyota_0 = 0.5
    time_to_restart_scenario = 60*2
    camera_height = 5
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

    stage_file_path = [STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2,STAGE_PATH_LATRUN_3]


    drone_path = "/World/drone"
    camera_path = "/World/camera_prim"
    camera2_path = "/World/camera2_prim"
    camera3_path = "/World/camera3_prim"
    ptz_dict = {'cam2':[camera2_path,'CamWest'], 'cam3': [camera3_path, 'CamEast']}

    radar_path = "/World/radar"
    toyota_path  = "/World/vehicles/toyota"
    tank_path = "/World/vehicles/tank"


    terrain_list = [f"/World/Terrain/Terrain_{i}" for i in range(len(stage_file_path))]
    utm_data = [Utils.open_coords_file(utm_data_piece) for utm_data_piece in [COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3]]
    poi = ['CamEast','CamWest','perimeter_part']
    asset_paths_names = ['logistics_part','drone_part']
    assets_names = ['toyota','drone']


    imgr = InputManager()
    input_server = InputServer(imgr)
    imapper = InputMapper(CFG_FILE)
    controller = Controller(imapper.cfg)

    mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
    'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}

    # load assets locations and paths
    asset_paths_objects = [GeoJSONLoader(utm_data[0], asset_path,GEOJSON_LATRUN) for asset_path in asset_paths_names]
    splines = {asset_name:path_object.spline_dict() for path_object,asset_name in zip(asset_paths_objects,assets_names)}
    poi_objects = [GeoJSONLoader(utm_data[0], poi_name,GEOJSON_LATRUN) for poi_name in poi]
    poi_points = {poi_name:poi_object.lat_lon_to_enu() for poi_object,poi_name in zip(poi_objects,poi)}
    geo_json_loader = GeoJSONLoader(utm_data[0], poi[0],GEOJSON_LATRUN)
    splines['drone']['spline_points'][:,2] = splines['drone']['spline_points'][:,2]*0 + hight_drone


    # load radar
    radar = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=poi_points['CamWest'][0,0:3])


    # Start background threads -----------------------------------------------------------
    # start server thread - this will listen to controller inputs
    threading.Thread(target=input_server.start, daemon=True).start()
    video_rb = RingBuffer(capacity=8, drop_policy="latest")
    VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread

    radar_rb = RingBuffer(capacity=512, drop_policy="latest")
    radar_pub = DetectionPublisherMQTT(radar_rb, radar.radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
    threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()


    scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ toyota_path,tank_path],[ WHITE_TOYOTA, TANK])}


    # load enviorment
    enviorment = Enviorment(stage_path='/World', light_path="/World/sky/DomeLight",  floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)
    new_origins = [Utils.new_origin_of_part(utm_data[0], utm_data_piece) for utm_data_piece in utm_data[1:]]
    [enviorment.load_prim_with_collision(f'{file_path}', stage_path) for file_path,stage_path in zip(stage_file_path, terrain_list)]
    [enviorment.translate_terrain(terrain, origin) for terrain,origin in zip(terrain_list[1:],new_origins)]
    enviorment.set_gravity(physics_path = "/World/Physics")
    world = World()
    world.reset()



    toyota = generate_asset_in_position(geo_json_loader,toyota_path,WHITE_TOYOTA, world, splines['toyota'],scale = 2, height=0)
    drone_camera, drone = generate_flying_cube( drone_path, splines['drone'], width, height)


    spline_assets = { 'toyota': toyota, 'drone': drone }


    ptz_cams = {cam_number : define_ptz_cam(cam_val[0], camera_height, poi_points, world, enviorment, width, height, cam_val[1]) for cam_number,cam_val  in ptz_dict.items()}



    spline_points_drone = splines['drone']['spline_points'][:,0:3]
    spline_points_der_drone = splines['drone']['spline_points_der']
    euler_initial_angles_drone = splines['drone']['euler_initial_angles']


    spline_points_der_toyota = splines['toyota']['spline_points_der']
    euler_initial_angles_toyota = splines['toyota']['euler_initial_angles']
    spline_points_toyota = splines['toyota']['spline_points'][:,0:3]





    # Create the curve prim
    spline_points_primiter = np.vstack(geo_json_loader.get_collisions( world, poi_points['perimeter_part'][:,0:3]))
    spline_points_primiter[:,2] = spline_points_primiter[:,2] + 2

    spline_points_primiter2 = spline_points_primiter.copy()
    spline_points_primiter2[:,2] = spline_points_primiter2[:,2] + 4


    Utils.add_curve_to_stage(enviorment.stage, spline_points_primiter,path='/World/BaseBorder_down',color=[Gf.Vec3f(1.0, 0.2, 0.2)])
    Utils.add_curve_to_stage(enviorment.stage, spline_points_primiter2,path='/World/BaseBorder_up',color=[Gf.Vec3f(0.0, 1.2, 0.2)])






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
            camera_orientation = controller.update_orientation(mapping, asset)
            camera_orientation = old_orientation+camera_orientation*dt
            old_orientation = camera_orientation.copy()


            asset = imapper.active_camera_target
            zoom_factor = controller.zoom_factor(mapping, asset)
            rot_flag = controller.rot_flag(mapping, asset)
            drone_camera.zoom_camera(zoom_factor)
            #-------------------------------------------------------------------------
            # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys


            time_tick_spline_asset(spline_assets['drone'],'drone',splines,controller,velocity_control)
            translation_toyota, orientation_toyota = time_tick_spline_asset(spline_assets['toyota'],'toyota',splines,controller,0)


            # translation_drone, orientation_drone = drone.get_position_and_orientation()
            # current_heading_vector = drone.get_local_orientation(orientation_drone)
            # angular_velocity_drone, linear_velocity_drone = controller.get_velocity_parameters_spline_path(translation_drone,current_heading_vector,splines['drone']['spline_points'],splines['drone']['spline_points_der'],'drone', kp = 200)

            # # Set linear velocity to follow the tangent, and angular velocity to turn
            # linear_velocity_drone = velocity_control if np.linalg.norm(velocity_control) > 0 else linear_velocity_drone
            # drone.set_velocity(linear_velocity_drone, local = False)
            drone_camera.transform_camera(camera_orientation,[0,0,0])


            # translation_toyota, orientation_toyota = toyota.get_position_and_orientation()
            # current_heading_vector = toyota.get_local_orientation(orientation_toyota)
            # angular_velocity_toyota, linear_velocity_toyota = controller.get_velocity_parameters_spline_path(translation_toyota,current_heading_vector,splines['toyota']['spline_points'],splines['toyota']['spline_points_der'],'toyota', kp = 200)
            # # Set linear velocity to follow the tangent, and angular velocity to turn
            # toyota.set_velocity(linear_velocity_toyota, local = False)
            # toyota.set_angular_velocity(np.array([0, 0, angular_velocity_toyota]), local = False) # Use world frame for simplicity



            # Get the final velocity for your radar
            velocity = toyota.get_linear_velocity(orientation_toyota)
            if controller.reset_asset(mapping, asset) :
                restart_time = now
                toyota.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,euler_initial_angles_toyota]),local = False)


            #___________________________________________________________________________________________________________

            if asset in ptz_dict:
                ptz_cams[asset][0].GetTargetPositionAttr().Set(0.0)   # ignore position
                ptz_cams[asset][0].GetTargetVelocityAttr().Set(rot_flag[2] * 50.0)  # deg/s
                ptz_cams[asset][1].GetTargetPositionAttr().Set(0.0)   # ignore position
                ptz_cams[asset][1].GetTargetVelocityAttr().Set(rot_flag[1] * 50.0)  # deg/s
                ptz_cams[asset][2].zoom_camera(zoom_factor)



            # check if its time to render / detect the next frame
            should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
            print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

            logic_time = time.perf_counter()

            # ------------------ rendering update ------------------
            world.step(render=should_render) # update the world simulation, render the frame if should_render is True
            
            target, false_alarm = radar.get_detections(translation_toyota, orientation_toyota, velocity, target_id = "car1")
            target = radar.check_if_targets_in_same_bin( [target], 'range')
            target = {k: np.concatenate([np.atleast_1d(d.get(k, [])).tolist() for d in target]) for k in set().union(*target)}
            detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])).tolist() for key in target.keys()}
            radar_rb.push(detection) # update the radar detection ring buffer 

            step_time = time.perf_counter()


            # if should_render is True, get the camera frame and add it to the queue for processing

            if should_render:
                passed_time += now - last_time
                last_time = now

                if asset in ptz_dict:
                    frame_rgb_bytes = ptz_cams[asset][2].frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = polar_plot)
                else:
                    frame_rgb_bytes = drone_camera.frame_in_bytes(None, az_deg=None, r_m=None, polar_plot = polar_plot)
                    
                if frame_rgb_bytes is not None:
                    video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
                render_time = time.perf_counter()
                print(f"Logic: {(logic_time - now)*1000:.2f}ms | Step: {(step_time - logic_time)*1000:.2f}ms | Render: {(render_time - step_time)*1000:.2f}ms")

            
            
            if now - last_time < time_to_wait_physics:
                time.sleep(time_to_wait_physics - (now - last_time))
        frame_count += 1
