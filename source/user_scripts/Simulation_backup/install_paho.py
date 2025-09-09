from isaacsim import SimulationApp

# Set the path below to your desired nucleus server
# Make sure you installed a local nucleus server before this
simulation_app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})


import omni.kit.pipapi as pipapi

# Pin a stable major—v1 is widely used and API-compatible with most examples.
# If you specifically want v2, change to "paho-mqtt==2.*".
ok = pipapi.install("paho-mqtt==1.6.1", module="paho.mqtt")
print("installed:", ok)

# Verify
import importlib.metadata as md
print("paho-mqtt version:", md.version("paho-mqtt"))


print("paho-mqtt version:", md.version("paho-mqtt"))
