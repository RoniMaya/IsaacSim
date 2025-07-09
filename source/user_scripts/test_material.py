from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({"headless": False})


from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import omni.kit.commands
import numpy as np
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import UsdShade, Gf, UsdGeom
# Initialize the high-level simulation world
world = World()
# world.scene.add_default_ground_plane()
# Get the current stage
stage = omni.usd.get_context().get_stage()

# Define the path for our ground plane
ground_prim_path = "/World/MyGround"
# ... (code to create and scale the plane remains the same) ...
omni.kit.commands.execute('CreateMeshPrim', prim_type='Plane', prim_path=ground_prim_path)
ground_prim = stage.GetPrimAtPath(ground_prim_path)
ground_prim.GetAttribute('xformOp:scale').Set(Gf.Vec3f(100, 100, 100))


# --- MODIFICATION START ---

# Define where the new material prim will be created in your stage
material_path = "/World/Looks/MyCarpetMaterial"

# Define the FULL PATH to your downloaded .mdl file on your computer
carpet_mdl_file_path = "/home/ronim/Downloads/Base_Materials_NVD@10013/Materials/2023_1/Base/Carpet/Carpet_Diamond_Olive.mdl"

# 3. Create a material prim by referencing your .mdl file
#    mtl_url is the source file on your disk.
#    mtl_path is where the new material prim is created in the USD stage.
omni.kit.commands.execute('CreateMdlMaterialPrim',
	mtl_url=carpet_mdl_file_path,
	mtl_name='Carpet_Diamond_Olive', # This is often the name of the material defined inside the MDL file
	mtl_path=material_path
)


material_prim = stage.GetPrimAtPath(material_path)
material = UsdShade.Material(material_prim)
ground_prim_mat_api = UsdShade.MaterialBindingAPI(ground_prim)
ground_prim_mat_api.Bind(material)
world.render()
cube_2 = world.scene.add(
    DynamicCuboid(
        prim_path="/World/new_cube_2",
        name="cube_1",
        position=np.array([5.0, 3, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
        linear_velocity=np.array([0, 0, 0]),
    )
)

cube_3 = world.scene.add(
    DynamicCuboid(
        prim_path="/World/new_cube_3",
        name="cube_2",
        position=np.array([0, 0, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0]),
    )
)

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 20.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

# world.scene.add_default_ground_plane()
world.reset()
camera.initialize()


import matplotlib.pyplot as plt

plt.ion()
figure, ax = plt.subplots()

i = 0
camera.add_motion_vectors_to_frame()

for i in np.arange(1,51,1):
    world.step(render=True)
    # print(camera.get_current_frame())
    if i % 50 == 0:
        points_2d = camera.get_image_coords_from_world_points(
            np.array([cube_3.get_world_pose()[0], cube_2.get_world_pose()[0]])
        )
        points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        # print(points_2d)
        # print(points_3d)
        # --- MODIFIED PLOTTING ---
        ax.clear() # Clear the previous image
        ax.imshow(camera.get_rgba()[:, :, :3])
        figure.canvas.draw()
        figure.canvas.flush_events()
        # -------------------------
        # print(camera.get_current_frame()["motion_vectors"])
        
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
    print(i)

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()