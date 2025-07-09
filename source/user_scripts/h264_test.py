from isaacsim import SimulationApp

# --- Simulation Setup ---
# It's good practice to start the simulation app first.
simulation_app = SimulationApp({"headless": True, "enable_cameras": True})

# --- Python Imports ---
# Group imports for better organization.
import numpy as np
import io
import time
import omni
import carb
import subprocess
import threading
from PIL import Image, ImageDraw, ImageFont

# --- FastAPI Imports ---
from fastapi import FastAPI, Request
from starlette.responses import StreamingResponse

# --- Isaac Sim Core Imports ---
from pxr import UsdLux, Gf
from isaacsim.core.prims import RigidPrim
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
import omni.kit.commands

# --- Isaac Sim World Initialization ---
world = World()
world.scene.add_default_ground_plane()
world.reset()
stage = omni.usd.get_context().get_stage()

# --- Add a Cube to the Scene ---
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        size=1.0,
        position=np.array([0, 0, 1.0]), # Start cube slightly above ground
        color=np.array([1.0, 0.0, 0.0])
    )
)
rigid_cube = RigidPrim("/World/cube")
pressed_keys = set()

# --- Add a Camera Looking at the Cube ---
camera = Camera(
    prim_path="/World/StreamCamera",
    position=np.array([0, 0, 50]), # Pulled back camera for a better view
    frequency=30, # Increased frequency for smoother video potential
    resolution=(640, 480), # Standard resolution
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True), # Angled down slightly
)
camera.initialize()

# --- Lighting Setup ---
# It's better to remove the default light and add your own for consistent results.
omni.kit.commands.execute('ChangeSetting',
    path='/rtx/scene/common/light/enableDefaultLight',
    value=False)
light_path = "/World/DomeLight"
dome_light = UsdLux.DomeLight.Define(stage, light_path)
dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
dome_light.GetIntensityAttr().Set(1500.0)

# ==============================================================================
# --- CORRECTED FFmpeg Setup for H.264 Encoding ---
# ==============================================================================
WIDTH, HEIGHT = 640, 480
RENDERING_FREQUENCY = 30
ffmpeg_process = subprocess.Popen([
    'ffmpeg',
    # Input Flags
    '-f', 'rawvideo',
    '-pix_fmt', 'rgb24',        # The pixel format of the data from Numpy
    '-s', f'{WIDTH}x{HEIGHT}',  # Input resolution
    '-r', str(RENDERING_FREQUENCY), # Match the FFmpeg input rate to our render frequency
    '-i', '-',                  # Input from stdin pipe

    # Output Flags
    '-c:v', 'libx264',          # Codec
    '-preset', 'ultrafast',     # Encoding speed vs. compression
    '-tune', 'zerolatency',     # CRITICAL: Optimizes for real-time streaming
    '-pix_fmt', 'yuv420p',      # IMPORTANT: Most web players require this pixel format.
    '-bufsize', '1000k',        # Video buffer size
    '-g', str(RENDERING_FREQUENCY * 2), # Group of Pictures (GOP) size (e.g., a keyframe every 2 seconds)
    '-f', 'mpegts',             # Muxer format for streaming
    '-'                         # Output to stdout pipe
], stdin=subprocess.PIPE, stdout=subprocess.PIPE)


# ==============================================================================
# --- FastAPI Server ---
# ==============================================================================
app = FastAPI()

@app.get("/video")
async def video_feed():
    """Streams the H.264 video feed."""
    def generate_stream():
        while True:
            # Read chunks of data directly from FFmpeg's stdout
            chunk = ffmpeg_process.stdout.read(4096)
            if not chunk:
                # This will happen if the ffmpeg process closes
                break
            yield chunk
    # The media type for MPEG-TS is 'video/mp2t'
    return StreamingResponse(generate_stream(), media_type="video/mp2t")

@app.post("/control")
async def press_key(request: Request):
    """Adds a key to the set of pressed keys."""
    command = (await request.body()).decode().strip().upper()
    pressed_keys.add(command)
    print(f"--- RECEIVED /control WITH KEY: '{command}' ---")

    return {"status": f"{command} pressed"}

@app.post("/release")
async def release_key(request: Request):
    """Removes a key from the set of pressed keys."""
    command = (await request.body()).decode().strip().upper()
    pressed_keys.discard(command)
    print(f"--- CURRENT PRESSED KEYS: {pressed_keys} ---")

    return {"status": f"{command} released"}

# --- Run FastAPI in a Background Thread ---
def start_fastapi():
    import uvicorn
    # Use 0.0.0.0 to make it accessible from other devices on the network
    uvicorn.run(app, host="0.0.0.0", port=8000)

server_thread = threading.Thread(target=start_fastapi, daemon=True)
server_thread.start()

print("\n" + "="*50)
print("Server started. Access the video stream at:")
print("http://127.0.0.1:8000/video")
print("="*50 + "\n")


# ==============================================================================
# --- Main Simulation Loop ---
# ==============================================================================
while simulation_app.is_running():
    # This steps the physics and renders a frame
    world.step(render=True)
    
    # Only process frames after the first world step, to ensure physics is initialized
    if world.is_playing():
        frame_rgba = camera.get_rgba()

        # Ensure the frame data is valid
        if frame_rgba is None or not frame_rgba.size:
            continue
        
        # --- Handle User Input ---
        velocity = np.array([0.0, 0.0, 0.0])
        speed = 5.0 # Give it a bit more speed
        if pressed_keys:
            direction = np.array([0.0, 0.0, 0.0])
            if "W" in pressed_keys: direction += np.array([1.0, 0.0, 0.0])
            if "S" in pressed_keys: direction += np.array([-1.0, 0.0, 0.0])
            if "A" in pressed_keys: direction += np.array([0.0, 1.0, 0.0])
            if "D" in pressed_keys: direction += np.array([0.0, -1.0, 0.0])
            
            # Normalize direction to prevent faster diagonal movement
            norm = np.linalg.norm(direction)
            if norm > 0:
                velocity = (direction / norm) * speed
        
        rigid_cube.set_linear_velocities(velocity)


        # --- Send Frame to FFmpeg ---
        # Drop the alpha channel and ensure it's the correct uint8 type
        frame_rgb = frame_rgba[:, :, :3].astype(np.uint8)
        
        try:
            # Write the raw RGB frame data to FFmpeg's stdin
            ffmpeg_process.stdin.write(frame_rgb.tobytes())
        except (BrokenPipeError, OSError):
            # This can happen if the reading end (FastAPI) closes the stream
            print("FFmpeg pipe broke. Shutting down.")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break

# --- Cleanup ---
print("Closing simulation.")
# Ensure the subprocess is terminated when the app closes
ffmpeg_process.stdin.close()
ffmpeg_process.terminate()
ffmpeg_process.wait()
simulation_app.close()

