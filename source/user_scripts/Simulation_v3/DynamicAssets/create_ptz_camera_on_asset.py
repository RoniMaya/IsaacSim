from Asset import Asset
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicCylinder
import Utils
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysxSchema, Sdf
import Joints

# Make sure to have these imports
from pxr import Gf, UsdPhysics, Sdf
import numpy as np

def create_ptz_camera_on_asset(
    stage,
    prim_path,
    parent_prim_path,
    relative_translation,
    scale_base=[0.1, 0.1, 0.01],
    scale_middle=[0.1, 0.1, 0.2],
    scale_child=[0.2, 0.2, 0.1]
):
    """
    Creates a PTZ camera and attaches it to a parent rigid body asset using a Fixed joint.

    Args:
        stage: The USD stage.
        prim_path (str): The desired path for the camera, e.g., "/World/MyPTZ".
        parent_prim_path (str): The path to the moving asset to attach to, e.g., "/World/MovingBox".
        relative_translation (Gf.Vec3f): The position of the camera's base relative to the parent asset's frame.
        scale_base, scale_middle, scale_child (list): Scales for the camera parts.
    """
    # Use half-extents (from the scale) for all local position calculations
    hb = scale_base[2] / 2.0
    hm = scale_middle[2] / 2.0
    hc = scale_child[2] / 2.0
    gap = 0.005 # A small visual gap between parts

    # --- 1. Create the Camera Parts as Rigid Bodies ---
    # The prims will be created under the main prim_path for organization
    base_path = f"{prim_path}/base"
    middle_path = f"{prim_path}/middle"
    child_path = f"{prim_path}/child"

    # Get the parent prim to read its world position for initial setup
    parent_prim = stage.GetPrimAtPath(parent_prim_path)
    parent_world_transform = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    parent_world_pos = parent_world_transform.ExtractTranslation()
    
    # Calculate the initial world positions for each part for correct placement at frame 0
    # The positions are stacked on top of the parent's position + relative_translation
    z_base = parent_world_pos[2] + relative_translation[2] + hb
    z_middle = z_base + hb + gap + hm
    z_child = z_middle + hm + gap + hc

    # --- KEY CHANGE: Base is now a rigid body ---
    stage.DefinePrim(base_path, "Cube")
    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(base_path))
    camera_base = Asset(base_path, rigid_prim=True, scale=scale_base)
    camera_base.set_pose(translation=np.array([parent_world_pos[0], parent_world_pos[1], z_base]))
    camera_base.disable_gravity()

    stage.DefinePrim(middle_path, "Cube")
    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(middle_path))
    camera_middle = Asset(middle_path, rigid_prim=True, scale=scale_middle)
    camera_middle.set_pose(translation=np.array([parent_world_pos[0], parent_world_pos[1], z_middle]))
    camera_middle.disable_gravity()

    stage.DefinePrim(child_path, "Cube")
    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(child_path))
    camera_child = Asset(child_path, rigid_prim=True, scale=scale_child)
    camera_child.set_pose(translation=np.array([parent_world_pos[0], parent_world_pos[1], z_child]))
    camera_child.disable_gravity()

    # --- 2. Define the Joints ---
    # The Revolute joints for pan and tilt remain the same, using LOCAL coordinates
    joint_props_yaw = Joints.make_joint_spec(
        axis="Z",
        parent_pos=Gf.Vec3f(0, 0, hb),     # Top surface of base
        child_pos=Gf.Vec3f(0, 0, -hm),    # Bottom surface of middle
        lower_deg=-180, upper_deg=180,
        stiffness=0.0, damping=1000.0
    )
    joint_props_pitch = Joints.make_joint_spec(
        axis="Y",
        parent_pos=Gf.Vec3f(0, 0, hm),     # Top surface of middle
        child_pos=Gf.Vec3f(0, 0, -hc),    # Bottom surface of child
        lower_deg=-45, upper_deg=45,
        stiffness=0.0, damping=1000.0
    )

    drive_base = Joints.define_joint(camera_base.prim_path, camera_middle.prim_path, stage, f"{prim_path}/joints/base_yaw", joint_props_yaw)
    drive_middle = Joints.define_joint(camera_middle.prim_path, camera_child.prim_path, stage, f"{prim_path}/joints/middle_pitch", joint_props_pitch)

    # --- 3. KEY CHANGE: Create the Fixed Joint to attach the camera to the parent asset ---
    fixed_joint_path = f"{prim_path}/joints/fixed_to_parent"
    fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
    fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_prim_path)])
    fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path(camera_base.prim_path)])

    # Set the attachment point on the PARENT (body0)
    fixed_joint.GetLocalPos0Attr().Set(relative_translation)
    fixed_joint.GetLocalRot0Attr().Set(Gf.Quatf(1.0, 0, 0, 0)) # No relative rotation

    # Set the attachment point on the CAMERA BASE (body1) - its bottom surface
    fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, -hb))
    fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0, 0, 0)) # No relative rotation
    
    return drive_base, drive_middle