
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni
from pxr import Usd, UsdGeom
import omni.usd
from isaacsim.core.api import World
from pxr import UsdLux, Gf
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np
from isaacsim.core.prims import RigidPrim


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


while simulation_app.is_running():
    speed = 10
    # 1. Calculate the direction vector based on the keys currently in the set
    direction = np.array([0.0, 0.0, 0.0])
    if carb.input.KeyboardInput.A in pressed_keys:
        direction += np.array([0.0, 1.0, 0.0]) # Forward
    if carb.input.KeyboardInput.D in pressed_keys:
        direction += np.array([0.0, -1.0, 0.0]) # Backward
    if carb.input.KeyboardInput.W in pressed_keys:
        direction += np.array([1.0, 0.0, 0.0]) # Left
    if carb.input.KeyboardInput.S in pressed_keys:
        direction += np.array([-1.0, 0.0, 0.0]) # Right
        
    # 2. Normalize the direction vector and scale by speed to get velocity
    #    We check if the magnitude is non-zero to avoid a division by zero error.
    norm = np.linalg.norm(direction)
    if norm > 0:
        normalized_direction = direction / norm
        velocity = normalized_direction * speed
    else:
        # If no movement keys are pressed, velocity is zero
        velocity = np.array([0.0, 0.0, 0.0])

    rigid_cube.set_linear_velocities(np.expand_dims(velocity, axis=0), indices=[0])

    world.step(render=True)
