
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni
from pxr import Usd, UsdGeom
import omni.usd
from isaacsim.core.api import World
from pxr import UsdLux, Gf
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np
from isaacsim.core.prims import RigidPrim
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()
stage = omni.usd.get_context().get_stage()

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/cube",
        name="cube",
        position=np.array([0.0, 0, 0.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)


camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 50.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

camera.initialize()
rigid_cube = RigidPrim("/cube")
velocity = np.array([0.0, 0.0, 0.0])
pressed_keys = set()



def keyboard_event(event, *args, **kwargs):
    global pressed_keys
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed_keys.add(event.input)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if event.input in pressed_keys:
            pressed_keys.remove(event.input)   



appwindow = omni.appwindow.get_default_app_window()
input_inter = carb.input.acquire_input_interface()
input_inter.subscribe_to_keyboard_events(appwindow.get_keyboard(), keyboard_event)
camera.add_motion_vectors_to_frame()

from fastapi import FastAPI
import matplotlib.pyplot as plt
from starlette.responses import StreamingResponse
import io
import numpy as np
from PIL import Image
import time

app = FastAPI()


def generate_frames():
    for idx in range(1,100,1):
        # Advance Isaac Sim simulation
        world.step(render=True)

        # Move object based on key input
        direction = np.array([0.0, 0.0, 0.0])
        if carb.input.KeyboardInput.A in pressed_keys:
            direction += np.array([0.0, 1.0, 0.0])
        if carb.input.KeyboardInput.D in pressed_keys:
            direction += np.array([0.0, -1.0, 0.0])
        if carb.input.KeyboardInput.W in pressed_keys:
            direction += np.array([1.0, 0.0, 0.0])
        if carb.input.KeyboardInput.S in pressed_keys:
            direction += np.array([-1.0, 0.0, 0.0])

        if np.linalg.norm(direction) > 0:
            velocity = direction / np.linalg.norm(direction) * 10
        else:
            velocity = np.array([0.0, 0.0, 0.0])

        rigid_cube.set_linear_velocity(velocity)

        # Capture image from Isaac Sim camera
        if idx > 10:
            frame = camera.get_rgba()[:, :, :3]
            image = Image.fromarray(frame, "RGB")

            buffer = io.BytesIO()
            image.save(buffer, format="JPEG")
            jpeg_bytes = buffer.getvalue()

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpeg_bytes + b"\r\n"
            )

        time.sleep(1/10)  # ~10 FPS


@app.get("/video")
def video_feed():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")
