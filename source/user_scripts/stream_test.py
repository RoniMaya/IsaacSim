from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "enable_cameras": True})
from pxr import UsdLux, Gf

import numpy as np
import io
import time
import omni

from PIL import Image
from fastapi import FastAPI
from starlette.responses import StreamingResponse
import threading
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.prims import set_prim_property

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
import omni.kit.commands

# Isaac Sim setup
world = World()
world.scene.add_default_ground_plane()
world.reset()
stage = omni.usd.get_context().get_stage()


# Add a cube to the scene
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        size=1.0,
        position=np.array([0, 0, 0]),
        color=np.array([1.0, 0.0, 0.0])
    )
)

# Add a camera looking at the cube
camera = Camera(
    prim_path="/World/StreamCamera",
    position=np.array([0, 0, 50]),
    frequency=20,
    resolution=(500, 500),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)
camera.initialize()
omni.kit.commands.execute('ChangeSetting',
    path='/rtx/scene/common/light/enableDefaultLight',
    value=False)
light_path = "/World/DomeLight"
dome_light = UsdLux.DomeLight.Define(stage, light_path)
dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
dome_light.GetIntensityAttr().Set(1000.0)



latest_jpeg = None  # Global shared frame buffer

# MJPEG generator
def generate_frames():
    global latest_jpeg
    while True:
        if latest_jpeg:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                latest_jpeg + b"\r\n"
            )
        time.sleep(1 / 20)


# FastAPI server
app = FastAPI()

@app.get("/video")
def video_feed():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

# Run FastAPI in a background thread

def start_fastapi():
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)

server_thread = threading.Thread(target=start_fastapi, daemon=True)
server_thread.start()

# Keep Isaac Sim alive
idx = 0
while simulation_app.is_running():
        world.step(render=True)
        frame = camera.get_rgba()

        if frame is None or len(frame.shape) != 3 or frame.shape[2] < 3:
            continue  # Wait for the next valid frame
        frame = frame[:, :, :3]  # Drop alpha channel

        image = Image.fromarray(frame.astype(np.uint8), "RGB")
        
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        latest_jpeg = buffer.getvalue()
        cube.set_linear_velocity(np.array([1.0, 0.0, 0.0]))  # Reset velocity
