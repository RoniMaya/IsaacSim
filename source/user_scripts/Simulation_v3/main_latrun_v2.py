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
import omni.isaac.core.utils.numpy.rotations as rot_utils

import paho.mqtt.client as mqtt
import cv2
from GeoJSONLoader import GeoJSONLoader
from isaacsim.core.utils import rotations 
import Joints
from DynamicAssets import ptz
from PolarPlotReusable import PolarPlotReusable
from pxr import UsdShade, Sdf
from path_define import (
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON,STAGE_PATH_TEL_KUDNA,STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3,
    STAGE_PATH_LATRUN_5, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,WHITE_TOYOTA,TANK,MEHOLA,STAGE_PATH_LATRUN_4,COORDINATES_LATRUN_4,COORDINATES_LATRUN_5,HAMAS,RADAR,KELA,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS,COORDINATES_TK,COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3, GEOJSON_GS, 
    GEOJSON_TK, GEOJSON_LATRUN
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, STAGE_PATH_TEL_KUDNA, STAGE_PATH_LATRUN_1, STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, COORDINATES_TK, GEOJSON_GS, GEOJSON_TK)



def spawn_spline_asset(asset_dict):
    if asset_dict['usd_path'] is None:
        stage = omni.usd.get_context().get_stage()
        Utils.generate_transperant_cube(stage,asset_dict['path'], scale_box=asset_dict.get('cube_scale', [1,1,1]),opacity= asset_dict.get('opacity', 0.0))

    asset = Asset(asset_dict['path'], usd_load_path=asset_dict['usd_path'], rigid_prim=True, scale=asset_dict.get('scale', [1,1,1]))
    spline_points_toyota = np.vstack(Utils.get_collisions(  asset_dict['spline']['spline_points'],height= asset_dict['height']))
    asset.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,asset_dict['spline']['euler_initial_angles']]),local = False)
    return asset


def spawn_asset_in_position(asset_dict, orientation = 0.0,rigid_prim= True):
    asset = Asset(asset_dict['path'], usd_load_path=asset_dict['usd_path'], rigid_prim=rigid_prim, scale=asset_dict['scale'])
    spline_points_primiter = np.vstack(Utils.get_collisions(  asset_dict['poi_location'],height = asset_dict['height']))
    asset.set_pose(translation=np.array(spline_points_primiter[0][0:3]), orientation = np.array([0,0,orientation]),local = False)
    return asset



def define_ptz_camera( asset_dict, translation,orientation,translation_camera = np.array([0,0,-3])):

    if asset_dict.get('usd_path',None) is None:
        stage = omni.usd.get_context().get_stage()
        Utils.generate_transperant_cube(stage,asset_dict['path'])

    drone = Asset(asset_dict['path'], rigid_prim=True, usd_load_path = asset_dict.get('usd_path',None), scale=asset_dict.get('scale', [1,1,1]))
    drone.set_pose(translation=translation, orientation=orientation, local=False)
    drone.disable_gravity()
    camera = drone.add_camera(asset_dict['resolution'][0], asset_dict['resolution'][1], translation = translation_camera)
    return camera, drone


def add_visual_to_child(asset_dict,stage):
    if not stage.GetPrimAtPath("/World/Looks"):
        Asset("/World/Looks/MyMaterial", usd_load_path=RADAR, rigid_prim=False, scale=[2]*3)

    Asset(f'{asset_dict["path"]}/look', usd_load_path=asset_dict['usd_path'], rigid_prim=False, scale=asset_dict.get('scale', [1,1,1])) 
    mesh_prim = stage.GetPrimAtPath(f'{asset_dict["path"]}/look')
    material_prim = stage.GetPrimAtPath("/World/Looks/MyMaterial")
    material_binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
    material_binding_api.Bind(UsdShade.Material(material_prim))


# need to add position in POI

def time_tick_spline_asset(asset,asset_dict,controller,velocity_control):
    # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
    translation_asset, orientation_asset = asset.get_position_and_orientation()
    current_heading_vector = asset.get_local_orientation(orientation_asset)
    angular_velocity_asset, linear_velocity_asset = controller.get_velocity_parameters_spline_path(translation_asset,current_heading_vector,
                                                                                                   asset_dict['spline']['spline_points'][:,0:3],
                                                                                                   asset_dict['spline']['spline_points_der'],
                                                                                                   asset_dict['cfg'], kp = 200)

    # Set linear velocity to follow the tangent, and angular velocity to turn
    linear_velocity_asset = velocity_control if np.linalg.norm(velocity_control) > 0 else linear_velocity_asset
    asset.set_velocity(linear_velocity_asset, local = False)
    asset.set_angular_velocity(np.array([0, 0, angular_velocity_asset]), local = False) # Use world frame for simplicity
    return translation_asset, orientation_asset

def user_ptz_orientation(ptz_object,ptz_dict_cam,mapping, asset):
    # update the ptz pitch angle - move the camera in pitch
    ptz_object[0].transform_camera([0,ptz_dict_cam['rotation'][1],0],[0,0,0])

    #----------------------------------------------------------------------------

    # change yaw angle of the camera - moving the cube in yaw
    translation, orientation = ptz_object[1].get_position_and_orientation()
    angular_velocity_yaw = controller.update_orientation(mapping, asset)
    ptz_object[1].set_angular_velocity([np.array([0,0,angular_velocity_yaw[2]/15])], orientation)
    camera_orientation_ptz,old_orientation_ptz = update_camera_orientation(mapping, asset,dt,ptz_dict_cam['rotation'])
    ptz_dict_cam['rotation'] = old_orientation_ptz
    return ptz_dict_cam





def user_camera_orientation(camera, controller,old_orientation,mapping, asset):
    camera_orientation,old_orientation = update_camera_orientation(mapping, asset,dt,old_orientation, pitch =True)
    zoom_factor = controller.zoom_factor(mapping, asset)
    camera.zoom_camera(zoom_factor)
    camera.transform_camera(camera_orientation,[0,0,0])
    return camera_orientation, old_orientation




def update_camera_orientation(mapping, asset,dt,old_orientation, pitch =True):    
    camera_orientation = controller.update_orientation(mapping, asset)
    camera_orientation = old_orientation+camera_orientation*dt
    old_orientation = camera_orientation.copy()

    return camera_orientation,old_orientation

def track_target(ptz_object,ptz_dict,controller, asset,target_elevation,target_angle):
    # move the camera in pitch 
    ptz_object[0].transform_camera([0,target_elevation,0],[0,0,0])
    # move the camera in yaw
    angular = controller.angular_velocity_p_controller(target_angle*np.pi/180,ptz_dict['rotation'][2]*np.pi/180, asset, kp = 0.2)
    translation, orientation = ptz_object[1].get_position_and_orientation()
    ptz_object[1].set_angular_velocity([np.array([0,0,angular*180/np.pi])], orientation)

    # translation, orientation = ptz_object[1].get_position_and_orientation()
    current_euler_angles = rot_utils.quats_to_euler_angles(orientation, degrees=True)
    # print(current_euler_angles, target_angle)
    ptz_dict['rotation'] = [0,target_elevation,current_euler_angles[2]]
    return ptz_dict








if __name__ == "__main__":


    width = 640
    height = 480
    physics_frequency = 60  # Hz
    rendering_frequency = 30  # Frames per second
    detection_frequency = 1  # Hz
    frame_count = 0
    passed_time = 0
    time_to_restart_scenario = 60*2
    last_time = time.monotonic()
    should_render= True
    should_detect = True
    next_render,next_detect = time.monotonic(),time.monotonic()
    render_every_n_frame = int(physics_frequency / rendering_frequency)
    text_for_image = {}
    show_pd_real_target = False
    polar_plot_radar2 = PolarPlotReusable(size=(220,220), r_max=400,color = 'g')
    time_to_wait_physics = 1.0 / physics_frequency
    coords_path = COORDINATES_LATRUN_1
    stage_path = STAGE_PATH_LATRUN_1

    stage_file_path = [STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2,STAGE_PATH_LATRUN_3,STAGE_PATH_LATRUN_4,STAGE_PATH_LATRUN_5]
    geo_coordinates_stage = [COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3,COORDINATES_LATRUN_4,COORDINATES_LATRUN_5]



    ptz_dict = {f'CamEast' : {'usd_path':None,'path':f"/World/cameras/CamEast", 'rotation': [0,0,0], 'rotation_ini': [0,0,0],'radar_angle':[0,90,0],
                               'radar_path':f"/World/radar0",'color':(0.0,1.0,0.0),'poi_location': None,'height':2,
                               'radar_plot': PolarPlotReusable(size=(220,220), r_max=400,color = 'g'),
                               'resolution': [width, height],'target':'hamas2'}}  # define the ptz cameras dictionary
    
    ptz_dict.update({f'CamWest' : {'usd_path':None,'path':f"/World/cameras/CamWest", 'rotation': [0,0,180],'rotation_ini': [0,0,180],
                                    'radar_angle':[0,90,180], 'radar_path':f"/World/radar1",'color':(1.0,0.0,0.0),'poi_location': None,'height':2,
                                      'radar_plot':PolarPlotReusable(size=(220,220), r_max=400),
                                      'resolution': [width, height],'target':'toyota'}})  # define the ptz cameras dictionary
    
    ptz_dict.update({f'CamNorth' : {'usd_path':None,'path':f"/World/cameras/CamNorth", 'rotation': [0,0,90],'rotation_ini': [0,0,90],
                                    'radar_angle':[0,90,-90], 'radar_path':f"/World/radar2",'color':(0.0,0.0,1.0),'poi_location': None,'height':2,
                                      'radar_plot':PolarPlotReusable(size=(220,220), r_max=400,color = 'b'),
                                      'resolution': [width, height],'target':'hamas1_1'}})  # define the ptz cameras dictionary
    ptz_cams = list(ptz_dict.keys())



    asset_toyota = {'name':'toyota','path':"/World/vehicles/toyota",'geo':'toyota', 'usd_path': WHITE_TOYOTA, 'scale': [2,2,2],'height':1, 'gravity': True,'cfg':'toyota'}
    asset_toyota_2 = {'name':'toyota2','path':"/World/vehicles/toyota2",'geo':'toyota2', 'usd_path': WHITE_TOYOTA, 'scale': [2,2,2],'height':1, 'gravity': True,'cfg':'toyota2'}
    asset_hamas1_1 = {'name':'hamas1_1','path':"/World/soldiers/hamas1_1",'geo':'hamas1_1', 'usd_path': HAMAS, 'scale': [0.5,0.5,0.5],'height':0.5,'gravity': False,'cfg':'hamas11'}
    asset_hamas1_2 = {'name':'hamas1_2','path':"/World/soldiers/hamas1_2",'geo':'hamas1_2', 'usd_path': HAMAS, 'scale': [0.5,0.5,0.5],'height':0.5,'gravity': False,'cfg':'hamas11'}
    asset_hamas1_3 = {'name':'hamas1_3','path':"/World/soldiers/hamas1_3",'geo':'hamas1_3', 'usd_path': HAMAS, 'scale': [0.5,0.5,0.5],'height':0.5,'gravity': False,'cfg':'hamas11'}

    asset_hamas2 = {'name':'hamas2','path':"/World/soldiers/hamas2", 'geo':'hamas2', 'usd_path': HAMAS, 'scale': [0.5,0.5,0.5],'height':0.5,'gravity': False,'cfg':'hamas'}
    asset_mehola = {'name':'mehola','path':"/World/mehola", 'geo':'mehola', 'usd_path': MEHOLA, 'scale': [5,5,5],'height':1}
    asset_mehola2 = {'name':'mehola2','path':"/World/mehola2", 'geo':'mehola2', 'usd_path': MEHOLA, 'scale': [5,5,5],'height':1,'orientation':80}
    asset_kela = {'name':'kela','path':"/World/soldiers/kela", 'geo':'kela', 'usd_path': KELA, 'scale': [0.5,0.5,0.5],'height':0.5, 'gravity': False,'cfg':'kela'}
    asset_tank = {'name':'tank','path':"/World/vehicles/tank", 'geo':'tank', 'usd_path': TANK, 'scale': [4,4,4],'height':1.5,'gravity': True,'cfg':'tank'}
    asset_tank2 = {'name':'tank2','path':"/World/vehicles/tank2", 'geo':'tank2', 'usd_path': TANK, 'scale': [4,4,4],'height':1.5,'gravity': True,'cfg':'tank2'}
    asset_perimeter_circle = {'name':'perimeter_square','path':"/World/perimeter_square", 'geo':'perimeter_square', 'usd_path': None,'height':15,'opacity':0.1,'cfg':'perimeter_circle'}
    asset_drone = {'name':'drone','path':"/World/drone", 'geo':'drone_part', 'usd_path': None,'height':20,'opacity':0.5, 'resolution': [width, height],'cfg':'drone'}

    track_with_radar = {'toyota':asset_toyota,'hamas1_1':asset_hamas1_1,'hamas1_2':asset_hamas1_2,'hamas1_3':asset_hamas1_3,'hamas2':asset_hamas2, 'toyota2':asset_toyota_2}

    assets_animation_list = [asset_toyota,asset_toyota_2,asset_hamas1_1,asset_hamas1_2,asset_hamas1_3,asset_hamas2,asset_kela]
    drone_list = [asset_drone]
    perimeters_list = [asset_perimeter_circle]
    assets_static_list = [asset_mehola,asset_mehola2,asset_tank,asset_tank2]

    utm_data = [Utils.open_coords_file(utm_data_piece) for utm_data_piece in geo_coordinates_stage] # load the UTM data for each stage


    imgr = InputManager()
    input_server = InputServer(imgr)
    imapper = InputMapper(CFG_FILE)
    controller = Controller(imapper.cfg)

    mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
    'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}

   
    # Start background threads -----------------------------------------------------------
    # start server thread - this will listen to controller inputs
    threading.Thread(target=input_server.start, daemon=True).start()
    video_rb = RingBuffer(capacity=8, drop_policy="latest")
    VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread

    # load enviorment------------------------------------------------------------------------------------
    terrain_list = [f"/World/Terrain/Terrain_{i}" for i in range(len(stage_file_path))] # define the terrain paths
    enviorment = Enviorment(stage_path='/World', light_path="/World/sky/DomeLight",  floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)
    new_origins = [Utils.new_origin_of_part(utm_data[0], utm_data_piece) for utm_data_piece in utm_data[1:]]
    [enviorment.load_prim_with_collision(f'{file_path}', stage_path) for file_path,stage_path in zip(stage_file_path, terrain_list)]
    [enviorment.translate_terrain(terrain, origin) for terrain,origin in zip(terrain_list[1:],new_origins)]
    world = World()
    world.reset()
    
    enviorment.set_gravity(physics_path = "/World/Physics",gravity_magnitude = 980)

    # load assets following a path------------------------------------------------------------------------------------
    objects_animation_dict = {}
    for asset in assets_animation_list:
        asset['spline'] = GeoJSONLoader(utm_data[0], asset['geo'],GEOJSON_LATRUN).spline_dict(get_collisions = True,height= asset['height'])
        objects_animation_dict[asset['name']] = spawn_spline_asset(asset)
        if asset.get('gravity', True) == False:
            objects_animation_dict[asset['name']].disable_gravity()


    # load a drone and its camera------------------------------------------------------------------------------------
    drones_dict = {}
    for drone_asset in drone_list:
        drone_asset['spline'] = GeoJSONLoader(utm_data[0], drone_asset['geo'],GEOJSON_LATRUN).spline_dict(get_collisions = True,height= drone_asset['height'])
        drone_camera, drone = define_ptz_camera( drone_asset,np.array(drone_asset['spline']['spline_points'][0]),
                                                np.array([0,0,drone_asset['spline']['euler_initial_angles']],))
        drones_dict[drone_asset['name']] = {'drone':drone,'camera':drone_camera}

    # load perimeter------------------------------------------------------------------------------------
    for perimeter in perimeters_list:
        perimeter['spline'] = GeoJSONLoader(utm_data[0], perimeter['geo'],GEOJSON_LATRUN).spline_dict(get_collisions = True,height= perimeter['height'])

        perimeter['poi_location'] = GeoJSONLoader(utm_data[0], perimeter['geo'],GEOJSON_LATRUN).lat_lon_to_enu()
        perimeter['poi_location'] = Utils.get_collisions(perimeter['poi_location'],height = perimeter['height'])

        Utils.add_curve_to_stage(enviorment.stage, np.vstack(perimeter['poi_location'])[:,0:3],
                                path=perimeter['path'],color=[Gf.Vec3f(0.1, 0.2, 0.5)],width = 0.5)

    # load static assets in position---------------------------------------------------------------------------------
    for asset in assets_static_list:
        asset['poi_location'] = GeoJSONLoader(utm_data[0], asset['geo'],GEOJSON_LATRUN).lat_lon_to_enu()
        asset['poi_location'] = Utils.get_collisions(asset['poi_location'],height = asset['height'])
        spawn_asset_in_position( asset, rigid_prim = False, orientation = asset.get('orientation',0.0))

    # load ptz cameras and radars---------------------------------------------------------------------------------
    ptz_objects,radars = {},{}
    for ptz_name in ptz_cams:
        ptz_dict[ptz_name]['poi_location'] = GeoJSONLoader(utm_data[0], ptz_dict[ptz_name]['path'].split('/')[-1],GEOJSON_LATRUN).lat_lon_to_enu() 
        ptz_dict[ptz_name]['poi_location'] =Utils.get_collisions(ptz_dict[ptz_name]['poi_location'],height = ptz_dict[ptz_name]['height'])
        ptz_objects[ptz_name] = define_ptz_camera(ptz_dict[ptz_name], ptz_dict[ptz_name]['poi_location'][0][0:3],
                                                   np.array([0,0,ptz_dict[ptz_name]['rotation'][2]]), np.array([0,0,ptz_dict[ptz_name]['height']]))
        # add_visual_to_child(ptz_dict[ptz_name],enviorment.stage)
        radar_origin = np.array([ptz_dict[ptz_name]['poi_location'][0][0],ptz_dict[ptz_name]['poi_location'][0][1],ptz_dict[ptz_name]['poi_location'][0][2] - 1])
        radars[ptz_name] = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=radar_origin, radar_angle= ptz_dict[ptz_name]['radar_angle'])


    radar_rb = RingBuffer(capacity=512, drop_policy="latest")
    radar_pub = DetectionPublisherMQTT(radar_rb, list(radars.values())[0].radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
    threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()
    [radar.plot_radar_direction( enviorment.stage, prim_path=f"{ptz_dict[ptz_name]['radar_path']}/radar{idx}_direction", width = 0.2, color=ptz_dict[ptz_name]['color']) for idx, (radar, ptz_name) in enumerate(zip(radars.values(), ptz_cams))]
    [ptz_objects[asset][0].transform_camera([0,0,0],[0,0,0]) for asset in ptz_objects.keys()]
    [Asset(f'{cam_path["path"]}/look', usd_load_path=RADAR, rigid_prim=False, scale=[1.5]*3) for cam_path in ptz_dict.values()]





    asset = 'cam1'
    dt = 1.0 / physics_frequency
    old_orientation = np.array([0,90,0])
    world.step(render=False)
    thermal_flag = False
    slave = False




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

            asset = imapper.active_camera_target

            # control the "drone" camera and update its position based on the spline path--------------------------------------------------
            if asset not in ptz_dict:
                # rotate the camera based on the keys
                camera_orientation, old_orientation = user_camera_orientation(drones_dict['drone']['camera'],controller, old_orientation,mapping, asset)
            # update the drone position based on the spline path
            translation_asset, orientation_asset = time_tick_spline_asset(drones_dict['drone']['drone'],asset_drone,controller,velocity_control)
            #_______________________________________________________________________________________________________________
            
            # move the assets based on the spline path--------------------------------------------------
            target_pose = {}
            for asset_animation in assets_animation_list:
                translation_asset, orientation_asset = time_tick_spline_asset(objects_animation_dict[asset_animation['name']],asset_animation,controller,0)
                if asset_animation['name'] in track_with_radar:
                    target_pose[asset_animation['name']] = {'orientation': orientation_asset, 'translation': translation_asset}

            # check if assets are in the radar FOV and get the detections--------------------------------------------------
            if asset in ptz_dict:
                target_list = {}
                for target_name, target_dict in track_with_radar.items():
                    object_target = objects_animation_dict[target_name]
                    velocity = object_target.get_linear_velocity(target_pose[target_name]['orientation'])
                    target_asset, false_alarm = radars[asset].get_detections(target_pose[target_name]['translation'], target_pose[target_name]['orientation'], velocity, target_id = target_name)
                    target_list[target_name] = target_asset

                target = radars[asset].check_if_targets_in_same_bin( list(target_list.values()), 'range')
                target = {k: np.concatenate([np.atleast_1d(d.get(k, [])).tolist() for d in target]) for k in set().union(*target)}
                detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])).tolist() for key in target.keys()}
                radar_rb.push(detection) # update the radar detection ring buffer

                zoom_factor = controller.zoom_factor(mapping, asset)
                ptz_objects[asset][0].zoom_camera(zoom_factor)
                slave = controller.toggle(slave, mapping, asset, 'slave')

                # move camera based on user input or track a target--------------------------------------------------
                if slave == False:
                    ptz_dict[asset] = user_ptz_orientation(ptz_objects[asset],ptz_dict[asset],mapping, asset)
                else: 
                    target_name = ptz_dict[asset]['target']
                    target_angle = target_list[target_name]['az_world'][0] if len(target_list[target_name]['az_world']) > 0 else ptz_dict[asset]['rotation'][2]
                    target_elevation = target_list[target_name]['elevation'][0] if len(target_list[target_name]['elevation']) > 0 else ptz_dict[asset]['rotation'][1]
                    ptz_dict[asset] = track_target(ptz_objects[asset],ptz_dict[asset],controller, asset,target_elevation,target_angle)


            # reset the scenario if needed--------------------------------------------------
            if controller.reset_asset(mapping, asset) :
                restart_time = now
                for asset_animation in assets_animation_list:
                    asset_object = objects_animation_dict[asset_animation['name']]
                    asset_object.set_pose(translation=np.array(asset_animation['spline']['spline_points'][0]), orientation = np.array([0,0,asset_animation['spline']['euler_initial_angles']]),local = False)

            # check if its time to render / detect the next frame
            should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
            logic_time = time.perf_counter()

            # ------------------ rendering update ------------------
            world.step(render=should_render) # update the world simulation, render the frame if should_render is True

            step_time = time.perf_counter()
            thermal_flag = controller.toggle(thermal_flag, mapping, asset, 'thermal')
            # if should_render is True, get the camera frame and add it to the queue for processing

            if should_render:
                passed_time += now - last_time
                last_time = now

                if asset in ptz_dict:
                    frame_rgb_bytes = ptz_objects[asset][0].frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = ptz_dict[asset]['radar_plot'], thermal = thermal_flag)
                else:
                    frame_rgb_bytes = drones_dict['drone']['camera'].frame_in_bytes(None, az_deg=None, r_m=None, polar_plot = None)

                if frame_rgb_bytes is not None:
                    video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
                render_time = time.perf_counter()
                # print(f"Logic: {(logic_time - now)*1000:.2f}ms | Step: {(step_time - logic_time)*1000:.2f}ms | Render: {(render_time - step_time)*1000:.2f}ms")

            
            
            if now - last_time < time_to_wait_physics:
                time.sleep(time_to_wait_physics - (now - last_time))
        frame_count += 1
