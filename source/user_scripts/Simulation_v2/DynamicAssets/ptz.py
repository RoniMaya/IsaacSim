from Asset import Asset
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicCylinder
import Utils
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf,PhysxSchema, Sdf
import Joints


def create_ptz_camera( stage,prim_path, translation, 
    scale_base = [0.2, 0.2, 0.1], scale_middle = [0.15, 0.15, 0.5], scale_child = [0.15, 0.15, 0.5]):


    # --- build joint specs with human-friendly degrees ---
    joint_props_yaw = Joints.make_joint_spec(
        axis="Z",
        parent_pos=Gf.Vec3f(0, 0, +scale_base[2]),   # top face of base (assuming 1 m cube)
        child_pos= Gf.Vec3f(0, 0, -scale_middle[2]),   # bottom face of middle
        lower_deg=-180, upper_deg=180,
        stiffness=0.0, damping=500.0,
        max_force=1000.0,
        target_vel_deg_s=60.0
    )

    joint_props_pitch = Joints.make_joint_spec(
        axis="Y",
        parent_pos=Gf.Vec3f(0, scale_middle[1], +scale_middle[2]),   # top face of middle
        child_pos= Gf.Vec3f(0, scale_child[1], -scale_child[2]),   # bottom face of child
        lower_deg=-45, upper_deg=45,
        stiffness=0.0, damping=500.0,
        max_force=1000.0,
        target_vel_deg_s=60.0
    )




    VisualCuboid(prim_path=f"{prim_path}/base", color=np.array([0, 0, 255]), scale=scale_base)
    camera_base = Asset(f"{prim_path}/base", rigid_prim = False)
    camera_base.set_pose(translation=np.array([translation[0], translation[1],translation[2]]), orientation = np.array([0,0,0]))

    camera_middle = stage.DefinePrim(f"{prim_path}/middle", "Cube")
    UsdPhysics.RigidBodyAPI.Apply(camera_middle)
    camera_middle = Asset(f"{prim_path}/middle", rigid_prim = True)
    camera_middle.set_pose(translation=np.array([translation[0], translation[1], translation[2]+scale_middle[2]]), orientation = np.array([0,0,0]))
    camera_middle.disable_gravity()

    camera_child = stage.DefinePrim(f"{prim_path}/child", "Cube")
    UsdPhysics.RigidBodyAPI.Apply(camera_child)
    camera_child = Asset(f"{prim_path}/child", rigid_prim = True)
    camera_child.set_pose(translation=np.array([translation[0], translation[1], translation[2]+scale_child[2] + scale_middle[2]]), orientation = np.array([0,0,0]))
    camera_child.disable_gravity()


    drive_base = Joints.define_joint( camera_base.prim_path, camera_middle.prim_path, stage, f"{prim_path}/joints/base", joint_props_yaw)
    drive_middle = Joints.define_joint(camera_middle.prim_path, camera_child.prim_path, stage, f"{prim_path}/joints/middle", joint_props_pitch)
    return drive_base, drive_middle
