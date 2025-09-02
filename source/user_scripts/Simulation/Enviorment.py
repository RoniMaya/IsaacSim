
from pxr import UsdLux, Gf
import omni.kit.commands
import omni.usd
from pxr import UsdShade, Gf, UsdGeom, UsdLux, Sdf, UsdPhysics, PhysicsSchemaTools

from omni.isaac.core.prims import GeometryPrim
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.prims import set_prim_property
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.kit.commands
import random

import numpy as np

class Enviorment():
    def __init__(self, world,light_path, floor_path = None, texture_sky = None, light_intensity = 1000):
        """
        Initializes the class by retrieving the current USD stage from the Omniverse context and assigning it to the 'stage' attribute.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.world = world
        self.floor_path = floor_path
        self.light_path = light_path
        self.dome_light = UsdLux.DomeLight.Define(self.stage, light_path)
        self.dome_light.GetIntensityAttr().Set(light_intensity)
        if floor_path is not None:
            self.add_colision_to_prim(floor_path)
        if texture_sky is not None:
            self.dome_light.GetTextureFileAttr().Set(texture_sky)


    def add_colision_to_prim(self,parent_prim_path):
        # Apply a single, aggregated collider to the parent
        # example parent_prim_path = "/World/ogmar_at_origin"
        parent_prim = self.stage.GetPrimAtPath(parent_prim_path)

        if parent_prim:
            UsdPhysics.CollisionAPI.Apply(parent_prim)
            print(f"Applied a single collider to {parent_prim.GetPath()}")


    def add_dome_light(self,light_path,intensity= 1500):
        """
        Adds a dome light to the stage at the specified path with the given intensity.
        This method disables the default light in the scene and creates a new dome light
        with white color and the specified intensity.
        Args:
            light_path (str): The USD path where the dome light will be created.
            intensity (float, optional): The intensity of the dome light. Defaults to 1500.
        Returns:
            None
        """

        omni.kit.commands.execute('ChangeSetting',
            path='/rtx/scene/common/light/enableDefaultLight',
            value=False)
        self.dome_light = UsdLux.DomeLight.Define(self.stage, light_path)
        self.dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        self.dome_light.GetIntensityAttr().Set(intensity)

    def bind_material(self,prim, mtl):
        # Setup a MaterialBindingAPI on the mesh prim
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
        # Use the constructed binding API to bind the material
        bindingAPI.Bind(mtl)

    def set_dome_direction(self,rotation_dict):
        [self.dome_direction(key,value) for key, value in rotation_dict.items()]
 


    def dome_direction(self, key,value):
        if key == 'X':
            self.dome_light.AddRotateXOp().Set(value)
        if key == 'Y':
            self.dome_light.AddRotateYOp().Set(value)
        if key == 'Z':
            self.dome_light.AddRotateZOp().Set(value)

        

