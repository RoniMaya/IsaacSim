from Asset import Asset
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicCylinder
import Utils
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf,PhysxSchema, Sdf
import Joints

# Make sure to have these imports
from pxr import Gf, UsdPhysics, Sdf
import numpy as np
# Assuming you have your Asset and Joints classes defined elsewhere

def create_ptz_camera(stage, prim_path, translation,
    scale_base = [1, 1, 1], scale_middle = [1,1,1], scale_child = [1,1,1]):

    # --- CORRECTED: Use half-extents for all calculations ---
    hb = 0.5*scale_base[2]
    hm = 0.5*scale_middle[2]
    hc = 0.5*scale_child[2]
    gap = 0.02

    # --- CORRECTED: Calculate world positions by stacking half-extents ---
    # Assuming translation[2] is the desired bottom position of the base
    z_base   = translation[2] + hb
    z_middle = z_base + hb + gap + hm
    z_child  = z_middle + hm + gap + hc

    # --- CORRECTED: build joint specs with proper LOCAL coordinates ---
    joint_props_yaw = Joints.make_joint_spec(
        axis="Z",
        # Local pos on parent (base): top surface
        parent_pos=Gf.Vec3f(0, 0, hb),
        # Local pos on child (middle): bottom surface
        child_pos=Gf.Vec3f(0, 0, -hm),
        lower_deg=-180, upper_deg=180,
        stiffness=0.0, damping=1000.0,
        max_force=1000.0,
        target_vel_deg_s=60.0
    )

    joint_props_pitch = Joints.make_joint_spec(
        axis="Y",
        # Local pos on parent (middle): top surface
        parent_pos=Gf.Vec3f(0, 0, hm),
        # Local pos on child (camera): bottom surface
        child_pos=Gf.Vec3f(0, 0, -hc),
        lower_deg=-45, upper_deg=45,
        stiffness=0.0, damping=1000.0,
        max_force=1000.0,
        target_vel_deg_s=60.0
    )

    # --- Create prims with CORRECTED world positions ---
    # camera_base = stage.DefinePrim(f"{prim_path}/base", "Cube")
    # UsdPhysics.RigidBodyAPI.Apply(camera_base)

    # This Asset class seems to wrap the prim, which is fine.
    camera_base = Asset(f"{prim_path}/base", rigid_prim=True, scale=scale_base)
    # Set pose using the new Z value
    camera_base.set_pose(translation=np.array([translation[0], translation[1], z_base]), orientation=np.array([0,0,0]))
    camera_base.disable_gravity()
    

    # camera_middle = stage.DefinePrim(f"{prim_path}/middle", "Cube")
    # UsdPhysics.RigidBodyAPI.Apply(camera_middle)
    camera_middle = Asset(f"{prim_path}/middle", rigid_prim=True, scale=scale_middle)
    # Set pose using the new Z value
    camera_middle.set_pose(translation=np.array([translation[0], translation[1], z_middle]), orientation=np.array([0,0,0]))
    camera_middle.disable_gravity()

    # camera_child = stage.DefinePrim(f"{prim_path}/child", "Cube")
    # UsdPhysics.RigidBodyAPI.Apply(camera_child)
    camera_child = Asset(f"{prim_path}/child", rigid_prim=True, scale=scale_child)
    # Set pose using the new Z value
    camera_child.set_pose(translation=np.array([translation[0], translation[1], z_child]), orientation=np.array([0,0,0]))
    camera_child.disable_gravity()

    base_prim = stage.GetPrimAtPath(f"{prim_path}/base")
    UsdPhysics.ArticulationRootAPI.Apply(base_prim)

    # Joint definitions remain the same, as they read from the corrected joint_props
    drive_base = Joints.define_joint(camera_base.prim_path, camera_middle.prim_path, stage, f"{prim_path}/joints/base", joint_props_yaw)
    drive_middle = Joints.define_joint(camera_middle.prim_path, camera_child.prim_path, stage, f"{prim_path}/joints/middle", joint_props_pitch)
    
    return drive_base, drive_middle,camera_middle