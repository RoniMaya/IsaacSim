import threading
from typing import Dict, Set, Any


class InputManager():
    """ Manages input states for multiple devices in a thread-safe manner."""
    def __init__(self):
        """
        Initializes the InputManager with a thread lock and an empty set of keys.
        """
        self._lock = threading.Lock()
        self._keys : Dict[str, Set[str]] = {}


    def set_key(self,device:str,key:str,pressed:bool):
        """
        Set the state of a key (pressed or released) for a specific device.
        Args:
            device (str): The device identifier (e.g., keyboard, gamepad).
            key (str): The key to set.
            pressed (bool): True if the key is pressed, False if released.
        """
        with self._lock:
            if device not in self._keys: # check if the device is not in the dictionary
                self._keys[device] = set() # if not, add it with an empty set
            if pressed: # if the key is pressed, add it to the set of pressed keys
                self._keys[device].add(key.upper())
            else: # if the key is released, remove it from the set of pressed keys
                self._keys[device].discard(key.upper())


    def snapshot(self):
        """
        Returns a snapshot of the current pressed keys for all devices.
        Returns:
            dict: A dictionary with device identifiers as keys and sets of pressed keys as values.
        """
        with self._lock:
            return {device: key for device, key in self._keys.items()}
