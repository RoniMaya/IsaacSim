from omni.isaac.kit import SimulationApp

# --- Simulation setup ---
# This dictionary is used to configure the simulation.
# "headless": False means the GUI will be shown.
config = {"headless": True}
simulation_app = SimulationApp(config)
import omni.kit.commands

from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
# from isaacsim.sensors import RTXSensor
from pxr import Gf
import numpy as np
import asyncio

# --- Main Simulation Class ---
class RadarSimulation:
    def __init__(self):
        # Create a new world for the simulation
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()

        # Setup the scene with radar and a target
        self.setup_scene()

        # Reset the world to ensure everything is initialized
        self._world.reset()

        # Variable to hold the radar sensor object
        import omni.kit.commands
        from pxr import Gf

        # This command creates the actual radar prim in the simulation stage
        omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            # The path where the radar will be created in the USD stage
            path="/MyRadar",

            # The parent prim for the radar
            parent="/World",

            # Position in meters
            translation=Gf.Vec3d(0, 0, 1.0),

            # Orientation as a quaternion (w, x, y, z) - this is no rotation
            orientation=Gf.Quatd(1, 0, 0, 0)
        )

        print("Radar prim created at /World/MyRadar")
    def setup_scene(self):
        """Creates the radar sensor and a target object."""
        
        # 1. Create the RTX Radar Sensor
        # Uses a high-level command to create an OmniRadar prim.
        # It's positioned at (0, 0, 1) and points forward along the X-axis.
        omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path="/Radar",
            parent="/World",
            translation=Gf.Vec3d(0, 0, 1.0),
            orientation=Gf.Quatd(1, 0, 0, 0), # No rotation
        )

        # 2. Create the Target Cube
        # This cube will be detected by the radar.
        # It's placed 5 meters in front of the radar.
        cuboid.VisualCuboid(
            prim_path="/World/TargetCube",
            position=np.array([5.0, 0, 1.0]),
            size=1.0,
            color=np.array([1.0, 0, 0]),
        )

    async def run_simulation(self):
        """The main asynchronous simulation loop."""
        
        # Start the simulation timeline
        self._world.play()
        print("Simulation started...")
        stage = omni.usd.get_context().get_stage()
        self.radar_prim = stage.GetPrimAtPath("/World/World_MyRadar")

        # Run simulation steps
        while simulation_app.is_running():
            # Perform a simulation step.
            # This advances physics and renders a frame if not headless.
            self._world.step(render=True)

            # Only get data if the simulation is running
            if self._world.is_playing():
                # Get the latest data from the radar sensor
                current_data = self.radar_prim.get_current_frame()

                # The data is a dictionary. We are interested in the point cloud.
                if 'radar_point_cloud' in current_data and len(current_data['radar_point_cloud']) > 0:
                    point_cloud_data = current_data['radar_point_cloud']
                    # Print the number of detection points
                    print(f"Timestamp: {current_data['time']:.2f} - Radar Detections: {point_cloud_data.shape[0]}")
            
            # Yield control to the event loop.
            # This is crucial for async functions to allow other tasks to run.
            await asyncio.sleep(0.01)



# --- Execution ---
if __name__ == "__main__":
    # Create an instance of our simulation class
    my_simulation = RadarSimulation()
    
    # Start the simulation event loop
    asyncio.ensure_future(my_simulation.run_simulation())
    
    # # Run the application
    asyncio.run(my_simulation.run_simulation())
    
    # # Cleanup
    # simulation_app.close()