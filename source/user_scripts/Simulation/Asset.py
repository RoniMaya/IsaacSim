
from isaacsim.core.prims import RigidPrim
import numpy as np
from omni.isaac.core.prims import XFormPrim
import omni.kit.commands
from pxr import UsdShade, Gf, UsdGeom
import omni.usd
from omni.isaac.core.utils.torch import quat_rotate
import warp as wp
import numpy as np
import Utils

class Asset():
    """Represents an asset in the simulation, managing its visual and physical properties."""
    def __init__(self, prim_path, rigid_prim = True):

        self.visual = XFormPrim(prim_path)
        self.rigid = RigidPrim(prim_path) if rigid_prim == True else None
        self.prim_path = prim_path


    def set_pose(self, translation , orientation = None ):
        """
        Sets the pose of the asset's visual representation.
        Args:
            translation (np.ndarray): A numpy array specifying the new position for the primitive in the form [x, y, z].
            orientation (np.ndarray, optional): A numpy array specifying the new orientation as a quaternion [x, y, z, w].
                If None, the orientation remains unchanged. Defaults to None.
        """

        self.visual.set_local_pose(translation=translation, orientation=orientation)
 

    def update_prim_scale(self, scale):
        """
        Updates the scale of the asset's visual representation.
        Args:
            scale (float or np.ndarray): A uniform scale factor (float) or a numpy array specifying non-uniform scaling [sx, sy, sz].
        """

        self.visual.set_local_scale(scale=scale)

    def set_velocity_local(self, velocity):
        """
        Sets the linear velocity of the asset in local coordinates in world coordinates.
        Args:
            velocity (np.ndarray): A numpy array specifying the local velocity vector [vx, vy, vz] in world coordinates.
        """
        _, orientation = self.rigid.get_world_poses() # get the current pose and orientation of the rigid body - in world coordinates
        world_velocity = Utils.quat_rotate_numpy(orientation[0], velocity)
        self.rigid.set_linear_velocities([world_velocity])


    def set_angular_velocity_local(self, angular_velocity):
        """
        Sets the angular velocity of the asset in local coordinates in world coordinates.
        Args:
            angular_velocity (np.ndarray): A numpy array specifying the local angular velocity vector [wx, wy, wz] in world coordinates.
        """
        _, orientation = self.rigid.get_world_poses() # get the current pose and orientation of the rigid body - in world coordinates
        angular_velocity = Utils.quat_rotate_numpy(orientation[0], angular_velocity[0])
        self.rigid.set_angular_velocities([angular_velocity])



