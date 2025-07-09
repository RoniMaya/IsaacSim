
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

class EnviormentClass():
    def __init__(self, world):
        """
        Initializes the class by retrieving the current USD stage from the Omniverse context and assigning it to the 'stage' attribute.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.world = world




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
        dome_light = UsdLux.DomeLight.Define(self.stage, light_path)
        dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        dome_light.GetIntensityAttr().Set(intensity)

    def bind_material(self,prim, mtl):
        # Setup a MaterialBindingAPI on the mesh prim
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
        # Use the constructed binding API to bind the material
        bindingAPI.Bind(mtl)


    def add_material(self,prim_path):

        mtl_created_list = []
        # Create a new material using OmniGlass.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniGlass.mdl",
            mtl_name="OmniGlass",
            mtl_created_list=mtl_created_list,
        )
        # Get reference to created material
        mtl_prim = self.stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(mtl_prim, "glass_color", Gf.Vec3f(0, 1, 0), Sdf.ValueTypeNames.Color3f)
        omni.usd.create_material_input(mtl_prim, "glass_ior", 1.0, Sdf.ValueTypeNames.Float)
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = self.stage.GetPrimAtPath(prim_path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)

    def add_texture(self, prim_path):
        mtl_created_list = []
        # Create a new material using OmniPBR.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniPBR.mdl",
            mtl_name="OmniPBR",
            mtl_created_list=mtl_created_list,
        )
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(
            mtl_prim,
            "diffuse_texture",
            "/home/ronim/Downloads/Base_Materials_NVD@10013/Materials/2023_2_1/Base/Carpet/Carpet_Diamond_Yellow/Carpet_Diamond_Yellow_BaseColor.png",
            Sdf.ValueTypeNames.Asset,
        )

        omni.usd.create_material_input(
        mtl_prim,
        "uv_transform",
        Gf.Vec4f(4.0, 4.0, 0.0, 0.0),  # (scale_u, scale_v, offset_u, offset_v)
        Sdf.ValueTypeNames.Float4,
        )
        
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)




    def add_mesh_as_ground(self, points,indices):

        # Create a mesh plane from points and indices

        # points, indices = self.generate_mountain_mesh(grid_size=10, terrain_width=10, max_height=5.0)

        mesh = UsdGeom.Mesh.Define(self.stage, '/World/plane')
        mesh.GetPointsAttr().Set(points)
        mesh.GetFaceVertexIndicesAttr().Set(indices)
        mesh.GetFaceVertexCountsAttr().Set([3] * len(indices))
        prim = GeometryPrim("/World/plane", collision=True)        # Set the materials path

        self.add_texture("/World/plane")

        # mtl_path = Sdf.Path(f"/World/Looks/PreviewSurface")
        # #Define the shader
        # shader = UsdShade.Shader.Define(self.stage, mtl_path.AppendPath("Shader"))
        # shader.CreateIdAttr("UsdPreviewSurface")
        # shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0,0,1)) 
        # shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.8)
        # shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        # shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.5)
        # shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.0)

        # mtl_path = Sdf.Path("/World/Looks/OmniPBR")

        # # Bind the material to the mesh
        # mtl = UsdShade.Material.Define(self.stage, mtl_path)
        # mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # # Call our bind material function. Note that we are passing a prim, not a mesh.
        # self.bind_material(mesh.GetPrim(), mtl)


    def add_ground(self):
        self.ground = FixedCuboid(
            prim_path="/World/ground",
            name="ground",
            position=np.array([0.0, 0.0, -0.05]),
            scale=np.array([100.0, 100.0, 0.1]),
            size=1.0,
        )

        # # Define where the new material prim will be created in your stage
        # material_path = "/World/Looks/MyCarpetMaterial"

        # # Define the FULL PATH to your downloaded .mdl file on your computer
        # carpet_mdl_file_path = "/home/ronim/Downloads/Base_Materials_NVD@10013/Materials/2023_1/Base/Carpet/Carpet_Diamond_Olive.mdl"

        # # 3. Create a material prim by referencing your .mdl file
        # #    mtl_url is the source file on your disk.
        # #    mtl_path is where the new material prim is created in the USD stage.
        # omni.kit.commands.execute('CreateMdlMaterialPrim',
        #     mtl_url=carpet_mdl_file_path,
        #     mtl_name='Carpet_Diamond_Olive', # This is often the name of the material defined inside the MDL file
        #     mtl_path=material_path
        # )


        # material_prim = self.stage.GetPrimAtPath(material_path)
        # material = UsdShade.Material(material_prim)
        # ground_prim = self.stage.GetPrimAtPath("/World/ground")

        # ground_prim_mat_api = UsdShade.MaterialBindingAPI(ground_prim)
        # ground_prim_mat_api.Bind(material)
            

    # def add_texture_to_ground(self, texture_path):
    #     Utils.apply_texture_to_prim(self.ground.prim_path, texture_path)