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
    CFG_FILE, STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON,STAGE_PATH_TEL_KUDNA,STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH,
    TEXTURE_SKY, CESIUM_TRANSFORM,WHITE_TOYOTA,TANK,MEHOLA,STAGE_PATH_LATRUN_4,STAGE_PATH_LATRUN_4,COORDINATES_LATRUN_4,HAMAS,RADAR,KELA,
    RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS,COORDINATES_TK,COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3, GEOJSON_GS, GEOJSON_TK, GEOJSON_LATRUN
)

print("Paths:", STAGE_PATH_OGMAR, STAGE_PATH_GAN_SHOMRON, STAGE_PATH_TEL_KUDNA, STAGE_PATH_LATRUN_1, STAGE_PATH_LATRUN_2, STAGE_PATH_LATRUN_3, CAR_ORANGE_ASSET_PATH, CAR_BLACK_ASSET_PATH, TEXTURE_SKY, CESIUM_TRANSFORM, RCS_FILE_PATH, RADAR_PROP_PATH, COORDINATES_GS, COORDINATES_TK, GEOJSON_GS, GEOJSON_TK)



# def define_ptz_cam(camera_path, poi_points, enviorment, width, height, camera_name, camera_height,orientation_base=  [0,0,0]):
#     camera_location = np.vstack(Utils.get_collisions(  poi_points[camera_name],height = camera_height))
#     drive_base_cam, drive_middle_cam = ptz.create_ptz_camera(enviorment.stage, prim_path = camera_path, 
#                             translation = np.array([camera_location[0,0], camera_location[0,1],camera_location[0,2]]),
#                             orientation_base = orientation_base)
#     camera = CameraClass(prim_path = f"{camera_path}/child/camera",orientation = np.array([0,0,0]),translation = [0.05,0,0.25],resolution = (width, height))
#     camera.camera.initialize()
#     return drive_base_cam, drive_middle_cam, camera


def spawn_spline_asset(asset_path,usd_load_path, splines,scale = [1,1,1], height=0,cube_scale = [1,1,1], opacity = 0):
    if usd_load_path is None:
        stage = omni.usd.get_context().get_stage()
        Utils.generate_transperant_cube(stage,asset_path, scale_box=cube_scale,opacity= opacity)

    asset = Asset(asset_path, usd_load_path=usd_load_path, rigid_prim=True, scale=scale)
    spline_points_toyota = np.vstack(Utils.get_collisions(  splines['spline_points'],height= height))
    asset.set_pose(translation=np.array(spline_points_toyota[0]), orientation = np.array([0,0,splines['euler_initial_angles']]),local = False)
    return asset


def spawn_asset_in_position(asset_path,usd_load_path, poi_points,asset_name,scale = 2, height=0,orientation = 0,rigid_prim= True):
    asset = Asset(asset_path, usd_load_path=usd_load_path, rigid_prim=rigid_prim, scale=[scale]*3)
    spline_points_primiter = np.vstack(Utils.get_collisions(  poi_points[asset_name],height = height))
    asset.set_pose(translation=np.array(spline_points_primiter[0][0:3]), orientation = np.array([0,0,orientation]),local = False)
    return asset








def define_ptz_camera( asset_path, width, height, translation,orientation,usd_load_path = None, scale = 1,translation_camera = np.array([0,0,-3])):

    if usd_load_path is None:
        stage = omni.usd.get_context().get_stage()
        Utils.generate_transperant_cube(stage,asset_path)

    drone = Asset(asset_path, rigid_prim=True, usd_load_path = usd_load_path, scale=[scale]*3)
    drone.set_pose(translation=translation, orientation=orientation, local=False)
    drone.disable_gravity()
    camera = drone.add_camera(width, height, translation = translation_camera)
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


def poi_with_collision(poi_points):
    point_with_collision = np.vstack(Utils.get_collisions(  poi_points))
    return point_with_collision


def update_camera_orientation(mapping, asset,dt,old_orientation, pitch =True):    
    camera_orientation = controller.update_orientation(mapping, asset)
    camera_orientation = old_orientation+camera_orientation*dt
    old_orientation = camera_orientation.copy()

    return camera_orientation,old_orientation




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

    stage_file_path = [STAGE_PATH_LATRUN_1,STAGE_PATH_LATRUN_2,STAGE_PATH_LATRUN_3,STAGE_PATH_LATRUN_4]
    geo_coordinates_stage = [COORDINATES_LATRUN_1,COORDINATES_LATRUN_2,COORDINATES_LATRUN_3,COORDINATES_LATRUN_4]



    ptz_dict = {f'CamEast' : {'path':f"/World/CamEast_prim", 'rotation': [0,0,0], 'rotation_ini': [0,0,0],'radar_angle':[0,90,0],
                               'radar_path':f"/World/radar0",'color':(0.0,1.0,0.0),'radar_plot': PolarPlotReusable(size=(220,220), r_max=400,color = 'g')}}  # define the ptz cameras dictionary
    ptz_dict.update({f'CamWest' : {'path':f"/World/CamWest_prim", 'rotation': [0,0,180],'rotation_ini': [0,0,180],
                                    'radar_angle':[0,90,180], 'radar_path':f"/World/radar1",'color':(1.0,0.0,0.0), 'radar_plot':PolarPlotReusable(size=(220,220), r_max=400)}})  # define the ptz cameras dictionary
    ptz_dict.update({f'CamNorth' : {'path':f"/World/CamNorth_prim", 'rotation': [0,0,90],'rotation_ini': [0,0,90],
                                    'radar_angle':[0,90,-90], 'radar_path':f"/World/radar2",'color':(0.0,0.0,1.0), 'radar_plot':PolarPlotReusable(size=(220,220), r_max=400,color = 'b')}})  # define the ptz cameras dictionary



    ptz_cams = list(ptz_dict.keys())

    poi_geofile = ptz_cams + ['tank','tank2','mehola','mehola2']
    height_poi = {'CamEast': 2, 'CamWest': 2,'CamNorth': 2, 'perimeter_circle': 15, 'tank': 2, 'drone': 20, 'toyota': 0,'tank2': 2,'mehola':1, 'hamas': 0.5,'mehola2':1,'kela':0.5}
    geo_spline_names = ['toyota','drone_part','hamas','perimeter_circle','kela']
    assets_names = ['toyota','drone','hamas','perimeter_circle','kela','tank','tank2','mehola','mehola2']

    drone_path = "/World/drone"
    camera_path = "/World/camera_prim"
    cameras_path = [f"/World/{camera_name}_prim" for camera_name in ptz_cams]
    toyota_path  = "/World/vehicles/toyota"
    tank_path = "/World/vehicles/tank"
    tank2_path = "/World/vehicles/tank2"
    mehola_path = "/World/mehola"
    mehola2_path = "/World/mehola2"
    hamas_path = "/World/soldiers/hamas"
    perimeter_circle_path = "/World/perimeter_circle"
    kela_path = "/World/soldiers/kela"

    terrain_list = [f"/World/Terrain/Terrain_{i}" for i in range(len(stage_file_path))] # define the terrain paths
    utm_data = [Utils.open_coords_file(utm_data_piece) for utm_data_piece in geo_coordinates_stage] # load the UTM data for each stage


    imgr = InputManager()
    input_server = InputServer(imgr)
    imapper = InputMapper(CFG_FILE)
    controller = Controller(imapper.cfg)

    mqtt_properties = {'mqtt_host': '127.0.0.1','mqtt_port': 1883,'mqtt_topic': '/device/magos/magos-service/platform/',
    'mqtt_qos': 0,'mqtt_retain': False, 'client_id': "radar_publisher"}

    
    asset_paths_objects = [GeoJSONLoader(utm_data[0], asset_path,GEOJSON_LATRUN) for asset_path in geo_spline_names] # load assets locations and paths
    splines = {asset_name:path_object.spline_dict() for path_object,asset_name in zip(asset_paths_objects,assets_names)} # create splines for each asset




    poi_objects = [GeoJSONLoader(utm_data[0], poi_name,GEOJSON_LATRUN) for poi_name in poi_geofile] # define the points of interest objects
    poi_points = {poi_name:poi_object.lat_lon_to_enu() for poi_object,poi_name in zip(poi_objects,poi_geofile)} # convert the points of interest to ENU coordinates per asset (not spline)

    # load radar    # radar = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=poi_points['CamWest'][0,0:3])


    # Start background threads -----------------------------------------------------------
    # start server thread - this will listen to controller inputs
    threading.Thread(target=input_server.start, daemon=True).start()
    video_rb = RingBuffer(capacity=8, drop_policy="latest")
    VideoPublisher(video_rb, width=width, height=height, target_fps=rendering_frequency)  # starts its own thread



    scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([ toyota_path,tank_path],[WHITE_TOYOTA, TANK])}


    # load enviorment
    enviorment = Enviorment(stage_path='/World', light_path="/World/sky/DomeLight",  floor_path=None, texture_sky = TEXTURE_SKY, light_intensity = 1000)
    new_origins = [Utils.new_origin_of_part(utm_data[0], utm_data_piece) for utm_data_piece in utm_data[1:]]
    [enviorment.load_prim_with_collision(f'{file_path}', stage_path) for file_path,stage_path in zip(stage_file_path, terrain_list)]
    [enviorment.translate_terrain(terrain, origin) for terrain,origin in zip(terrain_list[1:],new_origins)]
    

    world = World()
    world.reset()

    enviorment.set_gravity(physics_path = "/World/Physics",gravity_magnitude = 980)


    poi_points = {point_name:Utils.get_collisions(point_value,height = height_poi[point_name]) for point_name,point_value in poi_points.items()}
    radars = {ptz_name: None for ptz_name in ptz_cams}
    for ptz_name in ptz_cams:
        radars[ptz_name] = Radar(RCS_FILE_PATH, RADAR_PROP_PATH, radar_origin=poi_points[ptz_name][0][0:3], radar_angle= ptz_dict[ptz_name]['radar_angle'])

    radar_rb = RingBuffer(capacity=512, drop_policy="latest")
    radar_pub = DetectionPublisherMQTT(radar_rb, list(radars.values())[0].radar_properties, target_fps=1,mqtt_properties = mqtt_properties)
    threading.Thread(target=radar_pub.mqtt_publish, daemon=True).start()
    [radar.plot_radar_direction( enviorment.stage, prim_path=f"{ptz_dict[ptz_name]['radar_path']}/radar{idx}_direction", width = 0.2, color=ptz_dict[ptz_name]['color']) for idx, (radar, ptz_name) in enumerate(zip(radars.values(), ptz_cams))]



    toyota = spawn_spline_asset(toyota_path,WHITE_TOYOTA, splines['toyota'],scale = [2,2,2], height=height_poi['toyota'])


    # DynamicCuboid(prim_path=f"{hamas_path}", color=np.array([0, 255, 0]))
    hamas = spawn_spline_asset(hamas_path,HAMAS, splines['hamas'], height=height_poi['hamas'], scale = [0.5,0.5,0.5], opacity = 0)
    splines['hamas']['spline_points'][:,2] = splines['hamas']['spline_points'][:,2] + height_poi['hamas'] # set the hamas height
    splines['hamas']['spline_points'],splines['hamas']['spline_points_der'], _ = Utils.generate_spline_path_from_enu(splines['hamas']['spline_points'], spline_param = 3, num_samples = 500, add_z = 0)
    hamas.disable_gravity()




    splines['drone']['spline_points'] = np.vstack(Utils.get_collisions(  splines['drone']['spline_points'],height= height_poi['drone']))
    splines['drone']['spline_points'],splines['drone']['spline_points_der'], _ = Utils.generate_spline_path_from_enu(splines['drone']['spline_points'], spline_param = 3, num_samples = 500, add_z = 0)
    drone_camera, drone = define_ptz_camera( drone_path, width, height,np.array(splines['drone']['spline_points'][0]),
                                             np.array([0,0,splines['drone']['euler_initial_angles']],),usd_load_path=None)


    spawn_asset_in_position( tank_path, TANK, poi_points, 'tank', scale=4, height=height_poi['tank'])
    spawn_asset_in_position( tank2_path, TANK, poi_points, 'tank2', scale=4, height=height_poi['tank2'])
    spawn_asset_in_position( mehola_path, MEHOLA, poi_points, 'mehola', scale=5, height=height_poi['mehola'],rigid_prim = False)
    spawn_asset_in_position( mehola2_path, MEHOLA, poi_points, 'mehola2', scale=5, height=height_poi['mehola'],rigid_prim = False, orientation = 80)


    # DynamicCuboid(prim_path=f"{hamas_path}", color=np.array([0, 255, 0]))
    kela = spawn_spline_asset(kela_path,KELA, splines['kela'], height=height_poi['kela'], scale = [0.5,0.5,0.5], opacity = 0)
    splines['kela']['spline_points'][:,2] = splines['kela']['spline_points'][:,2] + height_poi['kela'] # set the kela height
    splines['kela']['spline_points'],splines['kela']['spline_points_der'], _ = Utils.generate_spline_path_from_enu(splines['kela']['spline_points'], spline_param = 3, num_samples = 500, add_z = 0)
    kela.disable_gravity()



    spline_assets = { 'toyota': toyota, 'drone': drone , 'hamas': hamas, 'kela': kela }




    spline_points_perimeter = np.vstack(Utils.get_collisions(  splines['perimeter_circle']['spline_points'],height= height_poi['perimeter_circle']))
    Utils.add_curve_to_stage(enviorment.stage, np.vstack(splines['perimeter_circle']['spline_points'])[:,0:3],
                             path='/World/perimeter_circle',color=[Gf.Vec3f(0.1, 0.2, 0.5)],width = 0.5)


    ptz_cams = {cam_name:  define_ptz_camera(cam_path, width, height,translation = np.array([poi_points[cam_name][0][0],poi_points[cam_name][0][1],
                                                                                             poi_points[cam_name][0][2] - 1]), 
                                                                                             orientation= np.array([0,0,ptz_dict[cam_name]['rotation'][2]]),
                                                                                             usd_load_path = None, scale=1,translation_camera= np.array([0,0,height_poi[cam_name]])) for cam_name,cam_path  in zip(ptz_cams, cameras_path)}
    [Asset(f'{cam_path}/look', usd_load_path=RADAR, rigid_prim=False, scale=[1.5]*3) for cam_path in cameras_path]


    Asset("/World/Looks/MyMaterial", usd_load_path=RADAR, rigid_prim=False, scale=[2]*3)
    stage = omni.usd.get_context().get_stage()
    mesh_prim = stage.GetPrimAtPath(f'{cameras_path[0]}/look')
    material_prim = stage.GetPrimAtPath("/World/Looks/MyMaterial")
    material_binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
    material_binding_api.Bind(UsdShade.Material(material_prim))


    [ptz_cams[asset][0].transform_camera([0,0,0],[0,0,0]) for asset in ptz_cams.keys()]


    target_angle = 0
    target_elevation = 0


    asset = 'cam1'
    dt = 1.0 / physics_frequency
    old_orientation = np.array([0,90,0])
    world.step(render=False)
    thermal_flag = False
    slave = False

    import math
    def yaw_from_quat_Zup(qwxyz):
        # qwxyz = [w, x, y, z] (Isaac/pxr usually WXYZ; adapt if yours is XYZW)
        w, x, y, z = qwxyz
        # Rotation matrix (Z-up)
        # R = [[1-2(y^2+z^2),   2(xy - wz),     2(xz + wy)],
        #      [  2(xy + wz), 1-2(x^2+z^2),    2(yz - wx)],
        #      [  2(xz - wy),   2(yz + wx),  1-2(x^2+y^2)]]
        R00 = 1 - 2*(y*y + z*z)
        R10 = 2*(x*y + w*z)
        yaw = math.atan2(R10, R00)  # shortest-path yaw in [-pi, pi]
        return yaw

    def wrap_to_pi(a):
        return (a + math.pi) % (2*math.pi) - math.pi


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

            if asset not in ptz_dict:

                camera_orientation = controller.update_orientation(mapping, asset)
                camera_orientation = old_orientation+camera_orientation*dt
                old_orientation = camera_orientation.copy()
                zoom_factor = controller.zoom_factor(mapping, asset)
                drone_camera.zoom_camera(zoom_factor)
                drone_camera.transform_camera(camera_orientation,[0,0,0])

            if asset in ptz_dict:

                zoom_factor = controller.zoom_factor(mapping, asset)
                ptz_cams[asset][0].zoom_camera(zoom_factor)


                slave = controller.enslave_camera(slave, mapping, asset)
                if slave == False:
                    ptz_cams[asset][0].transform_camera([0,camera_orientation[1],0],[0,0,0])

                    #----------------------------------------------------------------------------
                    # set the velocity, orientation and zoom of the "camera" cube
                    translation, orientation = ptz_cams[asset][1].get_position_and_orientation()
                    camera_orientation_kb = controller.update_orientation(mapping, asset)
                    ptz_cams[asset][1].set_angular_velocity([np.array([0,0,camera_orientation_kb[2]/15])], orientation)
                    camera_orientation,old_orientation_ptz = update_camera_orientation(mapping, asset,dt,ptz_dict[asset]['rotation'])
                    ptz_dict[asset]['rotation'] = old_orientation_ptz



                target_toyota, false_alarm = radars[asset].get_detections(translation_toyota, orientation_toyota, velocity, target_id = "toyota")
                target_hamas, false_alarm = radars[asset].get_detections(translation_hamas, orientation_hamas, velocity, target_id = "hamas")
                target = radars[asset].check_if_targets_in_same_bin( [target_toyota, target_hamas], 'range')

                if len(target) > 0:
                    target_angle = target[0]['az_world'][0] if len(target[0]['az_world']) > 0 else ptz_dict[asset]['rotation_ini'][2]
                    target_elevation = target[0]['elevation'][0] if len(target[0]['elevation']) > 0 else ptz_dict[asset]['rotation_ini'][1]
                    if slave == True:


                        ptz_cams[asset][0].transform_camera([0,target_elevation,0],[0,0,0])


                        angular = controller.angular_velocity_p_controller(target_angle*np.pi/180,ptz_dict[asset]['rotation'][2]*np.pi/180, asset, kp = 0.2)
                        # set the velocity, orientation and zoom of the "camera" cube
                        translation, orientation = ptz_cams[asset][1].get_position_and_orientation()
                        # camera_orientation_kb = controller.update_orientation(mapping, asset)
                        ptz_cams[asset][1].set_angular_velocity([np.array([0,0,angular*180/np.pi])], orientation)

                        translation, orientation = ptz_cams[asset][1].get_position_and_orientation()
                        current_euler_angles = rot_utils.quats_to_euler_angles(orientation, degrees=True)
                        print(current_euler_angles, target_angle)
                        ptz_dict[asset]['rotation'] = [0,target_elevation,current_euler_angles[2]]



                        # ptz_cams[asset][1].set_angular_velocity([np.array([0,0,camera_orientation_kb[2]/15])], orientation)
                        # camera_orientation,old_orientation_ptz = update_camera_orientation(mapping, asset,dt,ptz_dict[asset]['rotation'])
                        # ptz_dict[asset]['rotation'] = old_orientation_ptz

                        # ptz_dict[asset]['rotation'] = [0,target_elevation,target_angle]

                target = {k: np.concatenate([np.atleast_1d(d.get(k, [])).tolist() for d in target]) for k in set().union(*target)}
                detection = {"seq": frame_count, "time": round(time.time(),2)} | {key:np.hstack((target[key], false_alarm[key])).tolist() for key in target.keys()}
                radar_rb.push(detection) # update the radar detection ring buffer



            #-------------------------------------------------------------------------
            # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys


            time_tick_spline_asset(spline_assets['drone'],'drone',splines,controller,velocity_control)

            translation_toyota, orientation_toyota = time_tick_spline_asset(spline_assets['toyota'],'toyota',splines,controller,0)
            translation_hamas, orientation_hamas = time_tick_spline_asset(spline_assets['hamas'],'hamas',splines,controller,0)
            translation_kela, orientation_kela = time_tick_spline_asset(spline_assets['kela'],'kela',splines,controller,0)





            # Get the final velocity for your radar
            velocity = toyota.get_linear_velocity(orientation_toyota)
            if controller.reset_asset(mapping, asset) :
                restart_time = now
                toyota.set_pose(translation=np.array(splines['toyota']['spline_points'][0]), orientation = np.array([0,0,splines['toyota']['euler_initial_angles']]),local = False)
                hamas.set_pose(translation=np.array(splines['hamas']['spline_points'][0]), orientation = np.array([0,0,splines['hamas']['euler_initial_angles']]),local = False)



            #___________________________________________________________________________________________________________


            # check if its time to render / detect the next frame
            should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
            print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

            logic_time = time.perf_counter()

            # ------------------ rendering update ------------------
            world.step(render=should_render) # update the world simulation, render the frame if should_render is True


            step_time = time.perf_counter()
            thermal_flag = controller.thermal_camera(thermal_flag, mapping, asset)
            # if should_render is True, get the camera frame and add it to the queue for processing

            if should_render:
                passed_time += now - last_time
                last_time = now

                if asset in ptz_dict:
                    frame_rgb_bytes = ptz_cams[asset][0].frame_in_bytes(None, az_deg=detection.get('az_world', None), r_m=detection.get('range', None), polar_plot = ptz_dict[asset]['radar_plot'], thermal = thermal_flag)
                else:
                    frame_rgb_bytes = drone_camera.frame_in_bytes(None, az_deg=None, r_m=None, polar_plot = None)
                    
                if frame_rgb_bytes is not None:
                    video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing
                render_time = time.perf_counter()
                print(f"Logic: {(logic_time - now)*1000:.2f}ms | Step: {(step_time - logic_time)*1000:.2f}ms | Render: {(render_time - step_time)*1000:.2f}ms")

            
            
            if now - last_time < time_to_wait_physics:
                time.sleep(time_to_wait_physics - (now - last_time))
        frame_count += 1
