
from isaacsim.core.prims import RigidPrim,GeometryPrim
import numpy as np
from omni.isaac.core.prims import XFormPrim
import omni.kit.commands
from pxr import UsdShade, Gf, UsdGeom, Usd, UsdPhysics,PhysxSchema
import omni.usd
from omni.isaac.core.utils.torch import quat_rotate
import warp as wp
import numpy as np
import Utils
from scipy.spatial.transform import Rotation as R
from isaacsim.core.utils import rotations

class Asset():
    """Represents an asset in the simulation, managing its visual and physical properties."""
    def __init__(self, parent,category, name,scale = [1,1,1], rigid_prim = True, geometry_prim = False,usd_load_path = None,physics = {}, colision_type="convexDecomposition"):
        
        self.root_path     = f"/{parent}/{category}/{name}"
        self.geom_path     = f"{self.root_path}/Geom"
        self.colliders_path= f"{self.root_path}/Colliders"
        self.sensors_path  = f"{self.root_path}/Sensors"
        self.root_xf   = XFormPrim(self.root_path, scale=scale)   # set scale on root Xform
        self.visual_xf = XFormPrim(self.geom_path)
        self.colision_type = colision_type


        # --- visuals ----------------------------------------------------------
        if usd_load_path is not None:
            self.visual_xf.prim.GetReferences().AddReference(assetPath=usd_load_path)
            mesh_prims = self.get_mesh_children()
            [self.apply_collision(prim) for prim in mesh_prims]

        
        # --- physics ----------------------------------------------------------
        self.rigid = None
        if physics and physics.get("rigid", False):
            self.rigid = RigidPrim(self.root_path)  # attach PhysxRigidBodyAPI at the root
            self.rigid.enable_rigid_body_physics()
            if "mass" in physics:
                self.rigid.set_masses(physics["mass"])


        # self.rigid = RigidPrim(prim_path) if rigid_prim == True else None
        # self.geometry = GeometryPrim(prim_path) if geometry_prim == True else None

    def get_prim(self, prim_path = None):
        if prim_path == None:
           prim_path = self.root_path
        return self.root_xf.prim.GetPrimAtPath(prim_path)



    def disable_gravity(self):
        prim = self.get_prim()
        physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physxAPI.CreateDisableGravityAttr(True)


    def get_mesh_children(self):
        get_mesh_children = []
        prim = self.visual_xf.prim.GetPrimAtPath(self.root_path)
        all_prims = Usd.PrimRange(prim)
        for child_prim in all_prims:
            if child_prim.IsA(UsdGeom.Mesh):
                get_mesh_children.append(child_prim)
        return get_mesh_children



    def apply_collision(self, prim):

        UsdPhysics.CollisionAPI.Apply(prim) 
        self.collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        self.collision.GetApproximationAttr().Set(self.colision_type)

    def set_pose(self, translation , orientation = None ):
        """
        Sets the pose of the asset's visual representation.
        Args:
            translation (np.ndarray): A numpy array specifying the new position for the primitive in the form [x, y, z].
            orientation (np.ndarray, optional): A numpy array specifying the new orientation as a quaternion [x, y, z, w].
                If None, the orientation remains unchanged. Defaults to None.
        """
        if orientation is not None:
            orientation=rotations.euler_angles_to_quat(orientation)
        self.visual_xf.set_local_pose(translation=translation, orientation=orientation)

    def update_prim_scale(self, scale):
        """
        Updates the scale of the asset's visual representation.
        Args:
            scale (float or np.ndarray): A uniform scale factor (float) or a numpy array specifying non-uniform scaling [sx, sy, sz].
        """

        self.visual_xf.set_local_scale(scale=scale)

    def set_velocity_local(self, velocity,orientation):
        """
        Sets the linear velocity of the asset in local coordinates in world coordinates.
        Args:
            velocity (np.ndarray): A numpy array specifying the local velocity vector [vx, vy, vz] in world coordinates.
        """
        world_velocity = Utils.quat_rotate_numpy(orientation, velocity)
        self.rigid.set_linear_velocities([world_velocity])


    def set_angular_velocity_local(self, angular_velocity,orientation):
        """
        Sets the angular velocity of the asset in local coordinates in world coordinates.
        Args:
            angular_velocity (np.ndarray): A numpy array specifying the local angular velocity vector [wx, wy, wz] in world coordinates.
        """
        angular_velocity = Utils.quat_rotate_numpy(orientation, angular_velocity[0])
        self.rigid.set_angular_velocities([angular_velocity])


    def get_position_and_orientation(self):
        """
        Returns the current position of the asset's visual representation.
        Returns:
            np.ndarray: A numpy array representing the current position [x, y, z] of the asset.
        """
        translation, orientation = self.rigid.get_world_poses()
        return translation[0], orientation[0]

    # def get_velocity(self):
    #     """
    #     Returns the current linear velocity of the asset.
    #     Returns:
    #         np.ndarray: A numpy array representing the current linear velocity [vx, vy, vz] of the asset.
    #     """
        
    #     return self.rigid.get_linear_velocities()[0]

    def get_linear_velocity(self, orientation):

        return self.rigid.get_linear_velocities()[0]

    
    def get_local_orientation(self, orientation):
        orientation_quat_xyzw = orientation[[1, 2, 3, 0]]
        current_rot = R.from_quat(orientation_quat_xyzw)
        # Get the direction the car's front is pointing
        current_heading_vector = current_rot.apply([1, 0, 0])
        return current_heading_vector

    def set_velocity_world(self, world_velocity):
        """
        Sets the linear velocity of the asset in world coordinates.
        Args:
            velocity (np.ndarray): A numpy array specifying the velocity vector [vx, vy, vz] in world coordinates.
        """
        self.rigid.set_linear_velocities([world_velocity])

    def set_angular_velocity_world(self, angular_velocity):
        """
        Sets the angular velocity of the asset in world coordinates.
        Args:
            angular_velocity (np.ndarray): A numpy array specifying the angular velocity vector [wx, wy, wz] in world coordinates.
        """
        self.rigid.set_angular_velocities([angular_velocity])



    def set_pose_world(self, translation , orientation = None ):
        """
        Sets the pose of the asset's visual representation.
        Args:
            translation (np.ndarray): A numpy array specifying the new position for the primitive in the form [x, y, z].
            orientation (np.ndarray, optional): A numpy array specifying the new orientation as a quaternion [x, y, z, w].
                If None, the orientation remains unchanged. Defaults to None.
        """
        if orientation is not None:
            orientation=Utils.euler_to_quaternion(orientation)
        self.visual_xf.set_world_pose(position=translation, orientation=orientation)

