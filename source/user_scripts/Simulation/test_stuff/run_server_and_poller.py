# run_server_and_poller.py
import threading, time, json
import numpy as np

from InputManager import InputManager
from InputServer import InputServer
from InputMapper import InputMapper

# --- config ---
CFG_FILE  = '/home/ronim/isaacsim/source/user_scripts/Simulation/bindings.yaml'
HOST, PORT = "127.0.0.1", 9000

# --- Spin up server ---
imgr = InputManager()
server = InputServer(imgr)

# (Optional but recommended) allow browser fetch() from a file:// page or other origins
try:
    from fastapi.middleware.cors import CORSMiddleware
    server.app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"], allow_credentials=True,
        allow_methods=["*"], allow_headers=["*"],
    )
except Exception:
    pass

# Add loud endpoint logs (press/release) without touching your InputServer code:
# Wrap the existing routes after they've been created in server.start().
# We'll just print from InputManager via a tiny monkey patch.
_real_set_key = imgr.set_key
def _verbose_set_key(device: str, key: str, down: bool) -> None:
    _real_set_key(device, key, down)
    state = "⬇️  DOWN " if down else "⬆️  UP   "
    print(f"[key] {state} {device}:{key.upper()}  | currently pressed: {imgr.snapshot().get('keys', {})}")
imgr.set_key = _verbose_set_key  # monkey-patch for debugging

srv_thread = threading.Thread(target=server.start, kwargs={"host": HOST, "port": PORT}, daemon=True)
srv_thread.start()
print(f"✅ InputServer running on http://{HOST}:{PORT}   (endpoints: /press, /release)")

# --- Mapper + poll loop ---
imapper = InputMapper(CFG_FILE)

def _to_listable(mapping: dict):
    out = {}
    for tgt, m in mapping.items():
        d = dict(m)
        if isinstance(d.get("local_move"), np.ndarray):
            d["local_move"] = d["local_move"].tolist()
        if isinstance(d.get("rot_deg"), np.ndarray):
            d["rot_deg"] = d["rot_deg"].tolist()
        out[tgt] = d
    return out

_last_snap = {}
def poller():
    global _last_snap
    while True:
        snap = imgr.snapshot().get("keys", {})  # {'keyboard1': {'W','A',...}}
        # show changes in raw pressed keys (easy sanity check)
        if snap != _last_snap:
            print("[snapshot]", snap)
            _last_snap = {k: set(v) for k, v in snap.items()}

        # clear per tick to avoid accumulation
        imapper.mapping = {}
        mapping = imapper.calculate_mapping(snap)  # {target: {...}}
        if mapping:
            print("[mapping]", json.dumps(_to_listable(mapping), indent=2))
        time.sleep(0.1)

threading.Thread(target=poller, daemon=True).start()

# Keep main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Bye")
