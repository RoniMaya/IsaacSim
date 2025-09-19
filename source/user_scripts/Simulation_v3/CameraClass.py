# --- Python Imports ---
# Group imports for better organization.
import numpy as np
import io
import time
import omni
import carb
import subprocess
import threading
import queue  # IMPORTANT: For thread-safe communication
from PIL import Image,ImageDraw,ImageFont


# --- Isaac Sim Core Imports ---
from pxr import UsdLux, Gf
from isaacsim.core.prims import RigidPrim
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
import omni.kit.commands
import Utils
from Asset import Asset
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicCylinder











class CameraClass():
    def __init__(self, prim_path="/World/StreamCamera", translation = np.array([0, 0, 0]),frequency=30,resolution=(640, 480), orientation = np.array([0, 0, 0]), **kwargs):
        """
        Initializes the CameraClass with the specified parameters and loads the camera.
        Args:
            prim_path (str): The path where the camera primitive will be created.
            translation (tuple or list): The position of the camera in the scene.
            frequency (int or float): The update frequency of the camera.
            resolution (tuple): The resolution of the camera (width, height).
            orientation (tuple or list): The orientation of the camera in Euler angles (degrees).
            **kwargs: Additional keyword arguments to be passed to the Camera constructor.
        """

        self.camera_path = prim_path
        self.translation = translation
        self.frequency = frequency
        self.resolution = resolution
        self.orientation = orientation
        self.load_camera()



    def load_camera(self, **kwargs):
        """
        Initializes and loads a Camera object with the specified parameters.
        Keyword Args:
            **kwargs: Additional keyword arguments to be passed to the Camera constructor.
        Attributes Used:
            self.camera_path (str): The path where the camera primitive will be created.
            self.translation (tuple or list): The position of the camera in the scene.
            self.frequency (int or float): The update frequency of the camera.
            self.resolution (tuple): The resolution of the camera (width, height).
            self.orientation (tuple or list): The orientation of the camera in Euler angles (degrees).
        Side Effects:
            Sets self.camera to an instance of Camera initialized with the provided and class attributes.
        """

        
        self.camera = Camera(
        prim_path = self.camera_path,
        translation = self.translation,  
        frequency = self.frequency,
        resolution = self.resolution,
        orientation=rot_utils.euler_angles_to_quats(self.orientation, degrees=True),
        **kwargs
    )


    def transform_camera(self, rotation,translation):
        """
        Transforms the camera by setting its local pose with the specified rotation (deg) and translation.

        Args:
            rotation (list or tuple): The rotation angles (in degrees) to apply to the camera, typically as [roll, pitch, yaw].
            translation (list or tuple): The translation vector to set the camera's position, typically as [x, y, z].

        Notes:
            The rotation is converted from Euler angles (in degrees) to a quaternion before being applied.
        """

        self.camera.set_local_pose(translation= translation,orientation = rot_utils.euler_angles_to_quats(rotation, degrees=True))

        
    def zoom_camera(self, zoom_factor):
        """
        Adjusts the camera's field of view (FOV) based on the specified zoom factor.

        Args:
            zoom_factor (float): The factor by which to adjust the camera's FOV. A value greater than 1.0 zooms in, while a value less than 1.0 zooms out.
        """
        current_fov = self.camera.get_focal_length()
        new_fov = current_fov / zoom_factor
        self.camera.set_focal_length(new_fov)


    def get_world_pose(self):
        """
        Returns the camera's world position and orientation in Euler angles (degrees).

        Returns:
            tuple: (translation, orientation) where orientation is in Euler angles (degrees).
        """

        translation, orientation = self.camera.get_world_poses()
        return translation, rot_utils.quats_to_euler_angles(orientation, degrees=True)

    def get_intrinsic_camera(self):
        """
        Returns the intrinsic parameters of the camera.

        Returns:
            dict: A dictionary containing the intrinsic parameters of the camera.
        """
        return self.camera.get_intrinsics()

    def get_frame(self):
        return self.camera.get_rgba()


    def frame_in_bytes(self,text_to_add = None,az_deg=None, r_m=None,polar_plot = None):
        frame_rgba = self.get_frame()
        if frame_rgba is not None and frame_rgba.size > 0:

            if text_to_add is not None:
                frame_rgba = Image.fromarray(frame_rgba)
                I1 = ImageDraw.Draw(frame_rgba)
                font = ImageFont.truetype("DejaVuSansMono.ttf", 18)
                I1.text((28, 36), text_to_add, fill=(255, 255, 255), font = font)
                frame_rgba = np.array(frame_rgba)

        # Add the polar inset if data provided
            if az_deg is not None and r_m is not None:
                frame_rgba = Image.fromarray(frame_rgba)
                rgba = polar_plot.update(az_deg, r_m)
                inset = Image.fromarray(rgba, mode="RGBA")
                frame_rgba.paste(inset, (20,20), inset)
                frame_rgba = np.array(frame_rgba)
            return frame_rgba[:, :, :3].tobytes()





        
        
        
        