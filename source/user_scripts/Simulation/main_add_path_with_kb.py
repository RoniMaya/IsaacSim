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


width = 640
height = 360
physics_frequency = 120  # Hz
rendering_frequency = 60  # Frames per second
detection_frequency = 2  # Hz

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
stage_path = os.path.join(os.path.expanduser("~"), "Documents", "cesium","textured_model", "z_upv2.usd")
tree_asset_path = "http://127.0.0.1:8080/omniverse://127.0.0.1/NVIDIA/Assets/Vegetation/Trees/Colorado_Spruce.usd"
car_asset_path = "/home/ronim/Documents/assets/car/car_v7.usd"

tree_path = "/World/tree"
car_path = "/World/car"



cesium_transformation = Utils.cesium_transformation('/home/ronim/Documents/cesium/ogmar/conf.JSON')
# find camera position from real world coordinates
camera_position_dms = {'lon':[35,22,35.57,'E'], 'lat':[30,55,45.43,'N'],'height':-270}
transformed_vertices = Utils.get_mesh_position_from_dms(camera_position_dms, cesium_transformation)


scale_axis = {asset_name:Utils.get_usd_props(asset_path) for asset_name, asset_path in zip([tree_path, car_path], [tree_asset_path, car_asset_path])}
open_stage(stage_path)



camera_position_dms1 = {'lon':[35,22,28.07,'E'], 'lat':[30,55,40.00,'N'],'height':-299}

camera_position_dms2 = {'lon':[35,22,36.06,'E'], 'lat':[30,55,43.17,'N'],'height':1.88}
# camera_position_dms3 = {'lon':[35,22,35.09,'E'], 'lat':[30,55,42.64,'N'],'height':2.41}

transformed_vertices1 = Utils.get_mesh_position_from_dms(camera_position_dms1, cesium_transformation)
transformed_vertices2 = Utils.get_mesh_position_from_dms(camera_position_dms2, cesium_transformation)
# transformed_vertices3 = Utils.get_mesh_position_from_dms(camera_position_dms3, cesium_transformation)
 

print("initilizing radar")
rcs_file_path = '/home/ronim/Documents/radar_sim/radar_rcs_maps/rcs_ford_raptor_1.pkl'
radar_prop_path = '/home/ronim/Documents/radar_sim/radar_parameters/MAGOS.yaml'
text_for_image = {}
delta_az = 18080
delta_el = 40
radar_angle = [0, 120, 100]               
origin_world_radar = np.array([transformed_vertices[0], transformed_vertices[1], transformed_vertices[2]+10])
radar = Radar(rcs_file_path, radar_prop_path, "ball", origin_world_radar, radar_angle, delta_az=delta_az, delta_el=delta_el)



world = World()
world.reset()

enviorment = Enviorment(world, light_path="/World/sky/DomeLight", floor_path="/World/z_upv2", texture_sky = '/home/ronim/Downloads/sly_chat.png', light_intensity = 1000)
enviorment.set_dome_direction({'Y':180, 'Z':180})


world.get_physics_context().set_gravity(-98.0)




DynamicCuboid(prim_path="/World/cube", color=np.array([0, 255, 0]))
cube = Asset("/World/cube")
cube.set_pose(translation=np.array([transformed_vertices[0], transformed_vertices[1], transformed_vertices[2]+10]), orientation = np.array([0,-30,-100]))
camera = CameraClass(prim_path = "/World/cube/camera",orientation = np.array([0, 0, 0]),translation = [0,0,0.0],resolution = (width, height))
camera.camera.initialize()


stage = omni.usd.get_context().get_stage()


car_prim_path = "/World/car"
tree_prim_path = "/World/tree"


# cube2 = Asset(car_prim_path)
cube2 = Asset(car_prim_path, usd_load_path=car_asset_path, rigid_prim=True, scale=[scale_axis[car_path][0]*1.5]*3)
tree = Asset(tree_prim_path, usd_load_path=tree_asset_path, rigid_prim=False, scale=[scale_axis[tree_path][0]]*3)



controller = Controller(imapper.cfg)


cube.disable_gravity()




list_v1 = list(transformed_vertices1 + np.array((0,0,20,0)))[0:3]
# list_v2 = list(transformed_vertices2)[0:3]


# pts = [Gf.Vec3f(list_v1), Gf.Vec3f(list_v2)]
# direction = 5*((Gf.Vec3f(list_v2) - Gf.Vec3f(list_v1))/np.linalg.norm(Gf.Vec3f(list_v2) - Gf.Vec3f(list_v1)))

# get raycast interface
raycast = omni.kit.raycast.query.acquire_raycast_query_interface()
# generate ray array
ray1 = omni.kit.raycast.query.Ray(list_v1, (0, 0, -1))
ray2 = omni.kit.raycast.query.Ray(list_v1, (0, 0, -1))
ray_array = [ray1, ray2]

seq_id = raycast.add_raycast_sequence()




for i in range(100):
    raycast.submit_ray_to_raycast_sequence_array(seq_id, ray_array)
    hit = raycast.get_latest_result_from_raycast_sequence_array(seq_id)
    world.step(render=True) # update the world simulation, render the frame if should_render is True

cube2.set_pose(translation=np.array(hit[2][0].hit_position), orientation = None)
# tree.set_pose(translation=np.array(hit[2][0].hit_position), orientation = None)
# cube.set_pose(translation=np.array((transformed_vertices[0], transformed_vertices[1],10)), orientation = None)



prim = cube2.get_prim()
mass_kg = 10000
mapi = UsdPhysics.MassAPI.Apply(prim)
mapi.GetMassAttr().Set(float(mass_kg))


frame_count = 0
passed_time = 0
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()



# path = Sdf.Path("/World/Paths/MainRoute")
# curves = UsdGeom.BasisCurves.Define(enviorment.stage, path)
# curves.CreateTypeAttr(UsdGeom.Tokens.cubic)
# curves.CreateWrapAttr(UsdGeom.Tokens.bspline)
# curves.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)  # or periodic; bspline/catmullRom also allow pinned
# curves.CreateCurveVertexCountsAttr(Vt.IntArray([len(pts)]))
# curves.CreatePointsAttr(Vt.Vec3fArray(pts))



# # Widths (constant, in cm). 10 cm is clearly visible.
# pv_api = UsdGeom.PrimvarsAPI(curves)
# pv_w = pv_api.CreatePrimvar("widths", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.constant)
# pv_w.Set(Vt.FloatArray([200.0]))

# # Optional: give it a color so it pops
# pv_c = pv_api.CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
# pv_c.Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.2, 0.2)]))

show_pd_real_target = False

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
        translation, orientation = cube2.get_position_and_orientation()
        cube2_velocity = controller.update_velocity_direction(mapping, 'cube2')
        cube2_orientation = controller.update_orientation(mapping, 'cube2')
        #----------------------------------------------------------------------------

        # set the velocity, orientation and zoom of the "camera" cube
        cube2.set_angular_velocity_local([cube2_orientation], orientation)
        cube2.set_velocity_local(np.array(cube2_velocity), orientation)
        velocity = cube2.get_linear_velocity(orientation)
        #___________________________________________________________________________________________________________

        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)

        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True
        target, false_alarm = radar.get_detections(translation, orientation, velocity)
        detection = {"seq": frame_count, "time": round(time.time(),2)} | target | false_alarm
        radar_rb.push(detection) # update the radar detection ring buffer 

        # if should_render is True, get the camera frame and add it to the queue for processing
        if should_render:
            passed_time += now - last_time
            last_time = now
            if print_detection:
                all_data_text = radar.print_detections(text_for_image,target, false_alarm, passed_time)
                if show_pd_real_target:
                    all_data_text = f"{all_data_text} \n real target{round(radar.target_range,2)} \n pd {round(radar.radar_properties['pd'],2)}"
            frame_rgb_bytes = camera.frame_in_bytes(all_data_text)
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing

    frame_count += 1
