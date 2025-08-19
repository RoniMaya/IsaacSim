from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({'headless': False})

from ServerManager import ServerManager
import subprocess
import threading

from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

from isaacsim.core.api.objects import DynamicCuboid,DynamicSphere
import numpy as np

from AssetController import AssetController
from CameraClass import CameraClass
from EnviormentClass import EnviormentClass
from PhysicsClass import PhysicsClass
from omni.isaac.core.prims import GeometryPrim
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics,Gf, Sdf
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.sensors.rtx import LidarRtx, get_gmo_data
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema




width = 640
height = 360
physics_frequency = 90  # Hz
rendering_frequency = 60  # Frames per second
render_every_n_frame = int(physics_frequency / rendering_frequency)
asset_speed = 10  # Speed of the asset in the simulation
delta_angle = 2


ffmpeg_process = subprocess.Popen([
    "ffmpeg",
    "-f", "rawvideo",
    "-pix_fmt", "rgb24",
    "-s", f"{width}x{height}",
    "-r", str(rendering_frequency),
    "-i", "-",
    "-c:v", "libx264",
    "-preset", "ultrafast",
    "-tune", "zerolatency",
    "-pix_fmt", "yuv420p",
    "-g", "30",
    "-keyint_min", "30",  # consistent keyframes every 30 frames
    "-sc_threshold", "0",  # disable scene-change keyframes
    "-bufsize", "50k",    # small buffer
    "-f", "rtsp",
    "-rtsp_transport", "tcp",  # add this
    "rtsp://localhost:8554/mystream"
], stdin=subprocess.PIPE)


server = ServerManager(ffmpeg_process,stream_http = False)



# open_stage("/home/ronim/Downloads/Showcases_Content_NVD@10011/Samples/Showcases/2023_2_1/IsaacWarehouse/IsaacWarehouse.usd")

# open_stage("//home/ronim/Documents/gauss_try/gauss_try.usd")
# open_stage("/home/ronim/Documents/cesium/ogmar_at_origin.usd")

# /home/ronim/Documents/gauss_try
from omni.isaac.core.physics_context import PhysicsContext


world = World()
enviorment = EnviormentClass(world)
world.get_physics_context().set_gravity(0.0)

world.scene.add_default_ground_plane()

dynamic_cube = DynamicCuboid(prim_path="/World/cube", color=np.array([0, 255, 0]))
cube = AssetController("/World/cube")

# DynamicCuboid(prim_path="/World/cube2", color=np.array([255, 0, 0]))
# cube2 = AssetController("/World/cube2")

cube.set_pose(translation=np.array([0, 0, 0.0]), orientation = None)

# attatch a camera to the cube
camera = CameraClass(prim_path = "/World/camera",orientation = np.array([0, 90, 0]),translation = [0,0,50.0],resolution = (width, height))

enviorment.add_dome_light(light_path="/dome_light",intensity= 1000)



# radar_config = args.config
sensor_attributes = {'omni:sensor:WpmDmat:scan:s001:velResMps': 50,'omni:sensor:WpmDmat:scan:s001:rangeNoiseSDev': 0.05
}


# Create the radar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about +Z so it faces warehouse shelves.
# Possible config options are Example.
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(0.70711, 0.0, 0.0, 0.70711),
    **sensor_attributes
)

stage = omni.usd.get_context().get_stage()

radar_prim = stage.GetPrimAtPath("/sensor")

cfar_attr = radar_prim.CreateAttribute("omni:sensor:WpmDmat:cfarMode", Sdf.ValueTypeNames.Token)
cfar_attr.Set("4D")
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

render_product = rep.create.render_product(sensor.GetPath(), [1, 1], name="RtxRadarOutput")

annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
annotator.initialize()
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()
annotator.attach([hydra_texture])



world.reset()
camera.camera.initialize()





# Start background threads -----------------------------------------------------------
# start server thread - this will start the FastAPI server and the RTSP stream
server_thread = threading.Thread(target=server.start, daemon=True)
server_thread.start()


# start encoding thread - this will encode frames from the queue and send them to the RTSP stream
encoding_thread = threading.Thread(target=server.encode_frames_from_queue, daemon=True)
encoding_thread.start()

writer = rep.writers.get("RtxRadar" + "DebugDrawPointCloud")
writer.attach([hydra_texture])


physics = PhysicsClass(speed=asset_speed)
frame_count = 0
while simulation_app.is_running():
    if world.is_playing():
        radar_data = annotator.get_data()
        # Read GenericModelOutput struct from buffer
        # print(radar_data['data'])

        # get pressed keys from the input manager and update the physics and camera
        pressed_keys = server.input_manager.get_pressed_keys() 

        # update the velocity, orientation and zoom of the "camera" cube based on the pressed keys
        velocity = physics.update_velocity_direction(pressed_keys) 
        camera_orientation = physics.update_orientation(pressed_keys, delta_angle = delta_angle)
        zoom_factor = camera.get_zoom_factor(pressed_keys)
        #----------------------------------------------------------------------------

        # set the velocity, orientation and zoom of the "camera" cube
        cube.set_angular_velocity_local([camera_orientation])
        cube.set_velocity_local(velocity)
        camera.zoom_camera(zoom_factor)
        #-------------------------------------------------------------------------

        # if the frame count is divisible by the render_every_n_frame, define should_render == True----------
        should_render = (frame_count % render_every_n_frame == 0)
        world.step(render=should_render) # update the world simulation, render the frame if should_render is True

        # if should_render is True, get the camera frame and add it to the queue for processing
        if should_render:

            gmo_buffer = radar_data["info"]["sensorOutputBuffer"]
            gmo = get_gmo_data(gmo_buffer)

            frame_rgb_bytes = camera.frame_in_bytes(f'{gmo.velocities}, {radar_data["data"]}')
            # frame_rgb_bytes.text((28, 36), "nice Car", fill=(255, 0, 0))

            # if frame_rgba is not None and frame_rgba.size:
                # frame_rgb_bytes = frame_rgba[:, :, :3].astype(np.uint8).tobytes()
            if frame_rgb_bytes is not None:
                server.add_frame_to_queue(frame_rgb_bytes)  # Add the frame to the queue for processing


    frame_count += 1
