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
from pxr import Usd, UsdGeom, UsdPhysics, Gf,PhysxSchema, Sdf
import omni



width = 640
height = 360
physics_frequency = 120  # Hz
rendering_frequency = 60  # Frames per second
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
VideoPublisher(video_rb)



radar_rb = RingBuffer(capacity=512, drop_policy="latest")

radar_pub = DetectionPublisher(ring_buffer=radar_rb, target_fps=1)
threading.Thread(target=radar_pub.start, daemon=True).start()

# open_stage("/home/ronim/Downloads/Showcases_Content_NVD@10011/Samples/Showcases/2023_2_1/IsaacWarehouse/IsaacWarehouse.usd")


# Add ogmar--------------------------------------------------------------------------------------------
# stage_path = os.path.join(os.path.expanduser("~"), "Documents", "cesium", "ogmar_at_origin.usd")
# open_stage(stage_path)

print("initilizing radar")
rcs_file_path = '/home/ronim/Documents/radar_sim/radar_rcs_maps/rcs_ford_raptor_1.pkl'
radar_prop_path = '/home/ronim/Documents/radar_sim/radar_parameters/MAGOS.yaml'
text_for_image = {}
delta_az = 80
delta_el = 30
radar_angle = [0, 90, 0]               
origin_world_radar = np.array([0, 0, 0])
# radar = Radar(rcs_file_path, radar_prop_path, "ball", origin_world_radar, radar_angle, delta_az=delta_az, delta_el=delta_el)


world = World()
enviorment = Enviorment(world, light_path="/World/sky/DomeLight", floor_path="/World/z_upv2", texture_sky = '/home/ronim/Downloads/sly_chat.png', light_intensity = 1000)
world.get_physics_context().set_gravity(0.0)

# default ground plane
world.scene.add_default_ground_plane()

DynamicCuboid(prim_path="/World/cube", color=np.array([0, 255, 0]))
cube = Asset("/World/cube")

DynamicCuboid(prim_path="/World/cube2", color=np.array([255, 0, 0]))
cube2 = Asset("/World/cube2")

VisualCuboid(prim_path="/World/cube_radar", color=np.array([0, 0, 255]))
# cube_radar = Asset("/World/cube_radar", rigid_prim = False, geometry_prim = True)

# the_prim_to_collide = cube_radar.visual.prim 
# UsdPhysics.CollisionAPI.Apply(the_prim_to_collide)




# cube_radar.set_pose(translation=origin_world_radar, orientation=None)
cube.set_pose(translation=np.array([0, 0, 0.0]), orientation = None)
cube2.set_pose(translation=np.array([5, 0, 0.0]), orientation = None)

# attatch a camera to the cube
camera = CameraClass(prim_path = "/World/cube/camera",orientation = np.array([0, 0, 0]),translation = [0,0,0.0],resolution = (width, height))
enviorment.add_dome_light(light_path="/dome_light",intensity= 1000)


world.reset()
camera.camera.initialize()
controller = Controller(imapper.cfg)


frame_count = 0
passed_time = 0
last_time = time.monotonic()
should_render= True
should_detect = True
next_render,next_detect = time.monotonic(),time.monotonic()

from pxr import Usd, UsdGeom, PhysxSchema, Sdf, Gf
stage = omni.usd.get_context().get_stage()
# Bodies ---------------------------------------------------------
base  = stage.DefinePrim("/World/Base", "Cube")
child = stage.DefinePrim("/World/Child", "Cube")

# Make BASE static (no RigidBodyAPI) so it anchors the joint:
# If you want collisions on base, you can do: UsdPhysics.CollisionAPI.Apply(base)

# Make CHILD dynamic so it can move:
UsdPhysics.RigidBodyAPI.Apply(child)

# Place them so the hinge can be co-located (simple test: same pose)
UsdGeom.XformCommonAPI(base).SetTranslate((0, 0, 0))
UsdGeom.XformCommonAPI(child).SetTranslate((0, 0, 4))

# Revolute joint -------------------------------------------------
rev = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path("/World/BaseToChildJoint"))
rev.CreateBody0Rel().SetTargets([base.GetPath()])
rev.CreateBody1Rel().SetTargets([child.GetPath()])

# Set axis explicitly (use "Z" for yaw-like motion)
rev.GetAxisAttr().Set("Y")

# Co-locate anchors (same point/orient on both bodies)
rev.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0.5, 1))
rev.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0.5, -1))
rev.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
rev.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

# Limit rotation to ±45 degrees around the Y axis
rev.CreateLowerLimitAttr().Set(-45)   # -45°
rev.CreateUpperLimitAttr().Set(45)   # +45°
# Drive (use PURE VELOCITY control) ------------------------------
drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
drive.CreateStiffnessAttr().Set(0.0)          # <-- critical: no position hold
drive.CreateDampingAttr().Set(2.0)
drive.CreateMaxForceAttr().Set(1000.0)        # give it authority
drive.CreateTargetVelocityAttr().Set(30.0)    # deg/s
# (Don't set TargetPosition if you want velocity control)






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
        cube2.set_velocity_local(cube2_velocity, orientation)
        velocity = cube2.get_linear_velocity(orientation)
        #___________________________________________________________________________________________________________

        # check if its time to render / detect the next frame
        should_render, next_render = Utils.check_time_for_action(now, next_render, rendering_frequency)
        print_detection, next_detect = Utils.check_time_for_action(now, next_detect, detection_frequency)


        # Spin at 30 deg/s (positive around the joint axis)
        drive.GetTargetPositionAttr().Set(0.0)   # ignore position
        drive.GetTargetVelocityAttr().Set(30.0)  # deg/s


        # ------------------ rendering update ------------------
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True

        # target, false_alarm = radar.get_detections(translation, orientation, velocity)
        # detection = {"seq": frame_count, "time": round(time.time(),2)} | target | false_alarm
        # radar_rb.push(detection) # update the radar detection ring buffer 

        # if should_render is True, get the camera frame and add it to the queue for processing
        if should_render:
            passed_time += now - last_time
            last_time = now

                # all_data_text = radar.print_detections(text_for_image,target, false_alarm, passed_time)
                # all_data_text = f"{all_data_text} \n real target {radar.target_range} \n pd {radar.radar_properties['pd']}"
            frame_rgb_bytes = camera.frame_in_bytes()
            if frame_rgb_bytes is not None:
                video_rb.push({"bytes": frame_rgb_bytes, "seq": frame_count})  # Add the frame to the queue for processing

    frame_count += 1
