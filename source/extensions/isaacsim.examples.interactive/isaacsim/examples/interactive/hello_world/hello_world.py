# test_01_basics.py
from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane, DynamicCuboid

world = World.instance()
if world is None:
    world = World(stage_units_in_meters=1.0)

if "ground" not in world.scene.object_registry:
    world.scene.add(GroundPlane(prim_path="/World/Ground", name="ground", z_position=0.0))

world.scene.add(DynamicCuboid(
    prim_path="/World/MyCube",
    name="my_cube",
    position=[0, 0, 1],
    size=0.5,
    color=[1.0, 0.0, 0.0]
))

world.reset()
