# --- Simulation Setup ---
# It's good practice to start the simulation app first.
from isaacsim import SimulationApp
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
import queue  # IMPORTANT: For thread-safe communication
from PIL import Image

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


PHYSICS_FREQUENCY = 120  # Hz. Higher values increase control responsiveness.
RENDERING_FREQUENCY = 30 # Hz. This is the target framerate for the video stream.

# ==============================================================================
# --- Thread-Safe State Management ---
# ==============================================================================
# Use a lock to prevent race conditions when reading/writing the `pressed_keys`
# set from the main simulation thread and the FastAPI server thread.
pressed_keys_lock = threading.Lock()
pressed_keys = set()


# ==============================================================================
# --- Simulation World Setup ---
# ==============================================================================
world = World()
world.scene.add_default_ground_plane()

# --- Add a Cube to the Scene ---
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        size=1.0,
        position=np.array([0, 0, 1.0]),  # Start cube slightly above ground
        color=np.array([1.0, 0.0, 0.0])
    )
)
rigid_cube = RigidPrim("/World/cube")

# --- Add a Camera Looking at the Cube ---
camera = Camera(
    prim_path="/World/StreamCamera",
    position=np.array([0, 0, 50]),  # Pulled back camera for a better view
    frequency=30,
    resolution=(640, 480),
    # Angle the camera to look down at the cube from its position
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
)

# --- Lighting Setup ---
# It's better to remove the default light and add your own for consistent results.
stage = omni.usd.get_context().get_stage()
omni.kit.commands.execute('ChangeSetting',
    path='/rtx/scene/common/light/enableDefaultLight',
    value=False)
light_path = "/World/DomeLight"
dome_light = UsdLux.DomeLight.Define(stage, light_path)
dome_light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
dome_light.GetIntensityAttr().Set(1500.0)

# Initialize world and camera *after* setting everything up
world.reset()
camera.initialize()


# ==============================================================================
# --- FFmpeg Setup for H.264 Encoding ---
# ==============================================================================
WIDTH, HEIGHT = 640, 480
ffmpeg_process = subprocess.Popen([
    "ffmpeg",
    "-f", "rawvideo",
    "-pix_fmt", "rgb24",
    "-s", f"{WIDTH}x{HEIGHT}",
    "-r", str(RENDERING_FREQUENCY),
    "-i", "-",
    "-c:v", "libx264",
    "-preset", "ultrafast",
    "-tune", "zerolatency",
    "-pix_fmt", "yuv420p",
    "-g", "30",
    "-keyint_min", "30",  # consistent keyframes every 30 frames
    "-sc_threshold", "0",  # disable scene-change keyframes
    "-bufsize", "50k",    # small buffer
    "-f", "rtsp",
    "-rtsp_transport", "tcp",  # add this
    "rtsp://localhost:8554/mystream"
], stdin=subprocess.PIPE)



# ==============================================================================
# --- FastAPI Server & Background Threads ---
# ==============================================================================
app = FastAPI()
frame_queue = queue.Queue(maxsize=2)  # A small buffer for frames

# @app.get("/video")
# async def video_feed():
#     """Streams the H.264 video feed from FFmpeg's output."""
#     def generate_stream():
#         while True:
#             # Read chunks of data directly from FFmpeg's stdout pipe
#             chunk = ffmpeg_process.stdout.read(4096)
#             if not chunk:
#                 break
#             yield chunk
#     # The media type for MPEG-TS is 'video/mp2t'
#     return StreamingResponse(generate_stream(), media_type="video/mp2t")

@app.post("/press")
async def press_key(request: Request):
    """Adds a key to the set of pressed keys in a thread-safe manner."""
    command = (await request.body()).decode().strip().upper()
    with pressed_keys_lock:
        pressed_keys.add(command)
    return {"status": f"{command} pressed"}

@app.post("/release")
async def release_key(request: Request):
    """Removes a key from the set of pressed keys in a thread-safe manner."""
    command = (await request.body()).decode().strip().upper()
    with pressed_keys_lock:
        pressed_keys.discard(command)
    return {"status": f"{command} released"}

def start_fastapi():
    """Runs the FastAPI server in a dedicated thread."""
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

def encode_frames_from_queue():
    """Continuously pulls frames from the queue and feeds them to FFmpeg."""
    while True:
        try:
            # This will block until a frame is available
            frame_bytes = frame_queue.get()
            ffmpeg_process.stdin.write(frame_bytes)
        except (BrokenPipeError, OSError):
            print("FFmpeg pipe broke. Stopping encoding thread.")
            break
        except Exception as e:
            print(f"An error occurred in the encoding thread: {e}")
            break

# Start background threads
server_thread = threading.Thread(target=start_fastapi, daemon=True)
server_thread.start()

encoding_thread = threading.Thread(target=encode_frames_from_queue, daemon=True)
encoding_thread.start()

print("\n" + "="*50)
print("Server started. Access the video stream at:")
print("http://127.0.0.1:8000/video")
print("To control the cube, send POST requests to /control and /release")
print("with the key (W, A, S, D) in the request body.")
print("="*50 + "\n")


# ==============================================================================
# --- Main Simulation Loop ---
# ==============================================================================

def handle_input_and_update_physics():
    """
    Checks for user input and applies forces to the cube.
    This function is called once per physics step.
    """
    with pressed_keys_lock:
        # Make a copy to minimize the time the lock is held
        current_keys = pressed_keys.copy()

    if not current_keys:
        # It's better to set velocity to zero only if it's not already zero
        # to avoid unnecessary physics updates.
        current_vel = rigid_cube.get_linear_velocities()
        if np.any(current_vel):
             rigid_cube.set_linear_velocities(np.array([0.0, 0.0, 0.0]))
        return

    direction = np.array([0.0, 0.0, 0.0])
    speed = 5.0  # Movement speed
    if "W" in current_keys: direction += np.array([1.0, 0.0, 0.0])
    if "S" in current_keys: direction += np.array([-1.0, 0.0, 0.0])
    if "A" in current_keys: direction += np.array([0.0, 1.0, 0.0])
    if "D" in current_keys: direction += np.array([0.0, -1.0, 0.0])

    # Normalize direction to prevent faster diagonal movement and apply speed
    norm = np.linalg.norm(direction)
    if norm > 0:
        velocity = (direction / norm) * speed
        rigid_cube.set_linear_velocities(velocity)
    else:
        # This case handles if keys are pressed but result in no movement (e.g. W and S)
        rigid_cube.set_linear_velocities(np.array([0.0, 0.0, 0.0]))


# --- Loop Execution ---
frame_count = 0
# Tweak this value: Higher number = more responsive controls, but choppier video.
# A value of 3 means 1 render frame for every 3 physics steps.
RENDER_EVERY_N_FRAMES = int(PHYSICS_FREQUENCY / RENDERING_FREQUENCY)

last_time_check = time.time()


while simulation_app.is_running():
    # Only process frames and input after the simulation starts playing
    if world.is_playing():
        # 1. ALWAYS handle input. This runs at the physics rate, making it responsive.
        handle_input_and_update_physics()

        # 2. Decide whether to render this frame or just step physics
        should_render = (frame_count % RENDER_EVERY_N_FRAMES == 0)
        
        # 3. Step the simulation
        world.step(render=should_render)

        # 4. If we rendered, get the image data and queue it for encoding.
        if should_render:
            frame_rgba = camera.get_rgba()
            if frame_rgba is not None and frame_rgba.size:
                frame_rgb_bytes = frame_rgba[:, :, :3].astype(np.uint8).tobytes()
                try:
                    # Use put_nowait to avoid blocking the simulation loop.
                    frame_queue.put_nowait(frame_rgb_bytes)
                except queue.Full:
                    # This is okay. It means the video stream is lagging behind the
                    # simulation, and we're choosing to drop a frame to keep sim running smoothly.
                    pass
        
        frame_count += 1


        # Optional: Print the actual physics FPS to the console for debugging.
        # This will show you how fast the main loop is actually running.
        if frame_count % PHYSICS_FREQUENCY == 0:
            current_time = time.time()
            elapsed = current_time - last_time_check
            fps = PHYSICS_FREQUENCY / elapsed
            print(f"Physics Steps per Second: {fps:.2f}")
            last_time_check = current_time

    else:
        # If not playing, just step to keep the simulation visually updated
        world.step(render=True)


# --- Cleanup ---
print("Closing simulation.")
# Ensure the subprocess is terminated when the app closes
if ffmpeg_process.stdin:
    ffmpeg_process.stdin.close()
ffmpeg_process.terminate()
ffmpeg_process.wait()
simulation_app.close()
