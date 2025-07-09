import numpy as np
from isaacsim import SimulationApp

# Launch Isaac Sim with GUI
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# Initialize simulation world (1 unit = 0.1 meters)
world = World(stage_units_in_meters=0.1)
world.scene.add_default_ground_plane()

# Add a dynamic cube at 1 meter above ground (i.e., 10 units = 1.0)
fancy_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/fancy_cube",
        name="fancy_cube",
        position=np.array([0, 0, 10.0]),  # height = 1 meter
        scale=np.array([0.5, 0.5, 0.5]),  # scale in scene units
        size=1.0,
        color=np.array([0, 0, 1.0]),
    )
)

# Render initial state and reset
world.render()
world.reset()


def step_sim(xyz, cube, world):
    cube.set_world_pose(position=xyz, orientation=np.array([1.0, 0.0, 0.0, 0.0]))  # identity quaternion
    position, orientation = cube.get_world_pose()
    linear_velocity = cube.get_linear_velocity()
    world.step(render=True)  # advance simulation
    return np.hstack([position, orientation, linear_velocity])


# Move the cube through each position and store results
def on_play(xyz_list):
    x = np.arange(0, 100, 0.05)
    xyz_list = np.vstack([x, x, x]).T  # shape: (10, 3)

    for xyz in xyz_list:
        step_sim(xyz, fancy_cube, world)


world.add_physics_callback("my_play_callback", on_play)

    # Keep GUI open
while simulation_app.is_running():
    world.step(render=True)

