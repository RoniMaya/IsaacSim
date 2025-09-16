import sys
sys.path.insert(0, "/home/ronim/isaacsim/third_party/pydeps")

import os, sys, importlib, time
print("exe:", sys.executable)
print("PYTHONPATH[0]:", sys.path[0])
print("Top 5 sys.path:", sys.path[:5])

# 1) versions and path sanity
import numpy, scipy
print("NumPy:", numpy.__version__, "SciPy:", scipy.__version__)
# NumPy should NOT come from third_party/pydeps (we rely on Kit's 1.26.4)
print("numpy file:", numpy.__file__)
assert "/third_party/pydeps" not in numpy.__file__, "Remove numpy from pydeps to avoid ABI clashes."

# 2) basic SciPy ops (exercise compiled ext modules)
from scipy import linalg, signal, fft
A = [[3.0, 2.0],[2.0, 6.0]]
b = [2.0, -8.0]
x = linalg.solve(A, b)   # LAPACK
w = signal.get_window("hann", 16)  # signal
f = fft.rfft(w)          # FFT

# 3) IsaacSim core imports
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# 4) Start headless sim, spawn a cube, step a few frames
world = World()
cube = DynamicCuboid(
    prim_path="/World/TestCube",
    name="TestCube",
    position=(0,0,1),
    scale=(0.1,0.1,0.1),
)
for _ in range(5):
    world.step(render=False)
print("IsaacSim step OK")
print("SciPy OK:", x, f.shape)
print("ALL GOOD ✅")

simulation_app.close()

