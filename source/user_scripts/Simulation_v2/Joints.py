from pxr import Gf
import math
from dataclasses import dataclass
from typing import Literal
from pxr import Usd, UsdGeom, UsdPhysics, Gf,PhysxSchema, Sdf

Axis = Literal["X", "Y", "Z"]

@dataclass
class JointSpec:
    axis: Axis
    parent_pos: Gf.Vec3f
    child_pos:  Gf.Vec3f
    lower:  float
    upper:  float
    stiffness:  float
    damping:    float
    max_force:  float
    target_vel: float


def make_joint_spec(*, axis: Axis, parent_pos, child_pos,
                    lower_deg, upper_deg, stiffness, damping,
                    max_force, target_vel_deg_s) -> JointSpec:
    return JointSpec(
        axis=axis,
        parent_pos=parent_pos, child_pos=child_pos,
        lower=lower_deg,
        upper=upper_deg,
        stiffness=stiffness, damping=damping,
        max_force=max_force,
        target_vel=target_vel_deg_s
    )



def define_joint(parent_prim_path, connected_prim_path, stage, joint_path, joint_props):
    

    rev = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(joint_path))
    rev.CreateBody0Rel().SetTargets([parent_prim_path])
    rev.GetAxisAttr().Set(joint_props.axis)
    rev.CreateBody1Rel().SetTargets([connected_prim_path])

    rev.CreateLocalPos0Attr().Set(joint_props.parent_pos)
    rev.CreateLocalPos1Attr().Set(joint_props.child_pos)

    # Limit rotation to ±45 degrees around the Y axis
    rev.CreateLowerLimitAttr().Set(joint_props.lower)
    rev.CreateUpperLimitAttr().Set(joint_props.upper)
    # Drive (use PURE VELOCITY control) ------------------------------
    drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
    drive.CreateStiffnessAttr().Set(joint_props.stiffness)          # <-- critical: no position hold
    drive.CreateMaxForceAttr().Set(joint_props.max_force)        # give it authority
    drive.CreateTargetVelocityAttr().Set(joint_props.target_vel)    # deg/s
    drive.CreateDampingAttr().Set(joint_props.damping)   # higher value → stronger braking
    return drive

