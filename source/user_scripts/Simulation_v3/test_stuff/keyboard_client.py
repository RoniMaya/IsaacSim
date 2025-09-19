# keyboard_client.py
from pynput import keyboard
import requests

HOST = "http://127.0.0.1:9000"
DEVICE = "keyboard1"  # must match your YAML device name

def key_to_str(key):
    # Letters/digits -> uppercase char; NumPad digits -> same digit; ignore others
    try:
        if hasattr(key, "char") and key.char:
            return key.char.upper()
    except Exception:
        pass
    # Map a few special numpad keys if you use them
    NUMPAD = {
        keyboard.Key.num0:"0", keyboard.Key.num1:"1", keyboard.Key.num2:"2",
        keyboard.Key.num3:"3", keyboard.Key.num4:"4", keyboard.Key.num5:"5",
        keyboard.Key.num6:"6", keyboard.Key.num7:"7", keyboard.Key.num8:"8",
        keyboard.Key.num9:"9",
    }
    return NUMPAD.get(key)

def send(endpoint, key_str):
    try:
        requests.post(f"{HOST}/{endpoint}", json={"device": DEVICE, "key": key_str})
    except Exception as e:
        print("POST error:", e)

def on_press(key):
    ks = key_to_str(key)
    if ks:
        send("press", ks)

def on_release(key):
    ks = key_to_str(key)
    if ks:
        send("release", ks)
    if key == keyboard.Key.esc:
        # ESC to quit the client
        return False

print("🎹 Keyboard client: focus this terminal, press keys; ESC to quit.")
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
