
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

class AssetController():
    """
    AssetClass provides an interface for managing a visual asset and its associated rigid body in a simulation environment.

    """

    def __init__(self, prim_path, rigid_prim = True):

        self.visual = XFormPrim(prim_path)
        self.rigid = RigidPrim(prim_path) if rigid_prim == True else None
        self.prim_path = prim_path


    def set_pose(self, translation , orientation = None ):
        """
        Updates the transformation of the primitive by setting its local translation and orientation.
        Args:
            translation: The new translation (translation) to set for the primitive.
            orientation: The new orientation (rotation) to set for the primitive.
        """

        self.visual.set_local_pose(translation=translation, orientation=orientation)
 

    def update_prim_scale(self, scale):
        """
        Updates the scale of the primitive's visual representation.
        Args:
            scale (np.ndarray, optional): A numpy array specifying the new scale for the primitive in the form [x, y, z].
                Defaults to np.array([1, 1, 1]).
        Returns:
            None
        """

        self.visual.set_local_scale(scale=scale)

    def set_velocity_local(self, velocity):
        """
        Sets the linear velocity of the rigid body in world coordinates.
        Args:
            velocity (np.ndarray): The linear velocity vector to set for the rigid body (in local FoR).
        """     
        _, orientation = self.rigid.get_world_poses() # get the current pose and orientation of the rigid body - in world coordinates
        world_velocity = Utils.quat_rotate_numpy(orientation[0], velocity)
        self.rigid.set_linear_velocities([world_velocity])


    def set_angular_velocity_local(self, angular_velocity):
        _, orientation = self.rigid.get_world_poses() # get the current pose and orientation of the rigid body - in world coordinates
        angular_velocity = Utils.quat_rotate_numpy(orientation[0], angular_velocity[0])
        self.rigid.set_angular_velocities([angular_velocity])



