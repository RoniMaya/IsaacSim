# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--config", type=str, default="Example", help="Name of radar config.")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Example for creating a RTX lidar sensor and publishing PCL data
simulation_app = SimulationApp({"headless": True})
import carb
import omni
import omni.kit.viewport.utility
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

# enable ROS bridge extension
enable_extension("isaacsim.util.debug_draw")

simulation_app.update()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path() 
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

simulation_app.update()
# Loading the simple_room environment
# stage.add_reference_to_stage(
#     assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/background"
# )

# (Optional) Add a ground plane
# create_prim("/World/groundPlane", "Plane", attributes={"size": 20})

# Add a simple target cube
# create_prim("/World/TargetCube", "Cube", translation=(5, 0, 1), scale=(1, 1, 1))
world = World()
dynamic_cube = DynamicCuboid(prim_path="/World/cube", translation=(0, 5, 1), scale=(1, 1, 1))
rigid_cube = RigidPrim("/World/cube")

# dynamic_cube = DynamicCuboid(prim_path="/World/cube2", translation=(5, 0, 1), scale=(1, 1, 1))
# rigid_cube = RigidPrim("/World/cube2")

# dynamic_cube = DynamicCuboid(prim_path="/World/cube3", translation=(0, -5, 1), scale=(1, 1, 1))
# rigid_cube = RigidPrim("/World/cube3")

# dynamic_cube = DynamicCuboid(prim_path="/World/cube4", translation=(-5, 0, 1), scale=(1, 1, 1))
# rigid_cube = RigidPrim("/World/cube4")

world.scene.add_default_ground_plane()


simulation_app.update()

radar_config = args.config

# Create the radar sensor that generates data into "RtxSensorCpu"
# Sensor needs to be rotated 90 degrees about +Z so it faces warehouse shelves.
# Possible config options are Example.
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config=radar_config,
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(0.70711, 0.0, 0.0, 0.70711),
)
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

render_product = rep.create.render_product(sensor.GetPath(), [1, 1], name="RtxRadarOutput")

annotator = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
annotator.initialize()

# render_product_path = '/sensor'
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()
annotator.attach([render_product])

# Create the debug draw pipeline in the post process graph
writer = rep.writers.get("RtxRadar" + "DebugDrawPointCloud")
writer.attach([hydra_texture])

simulation_app.update()

simulation_context.play()
world.reset()

while simulation_app.is_running():
    simulation_app.update()
    radar_data = annotator.get_data()



# cleanup and shutdown
simulation_context.stop()
simulation_app.close()
