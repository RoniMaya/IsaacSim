
import yaml
from dataclasses import dataclass
import tempfile

import numpy as np


class InputMapper():
    """
    InputMapper maps input device keys to actions based on a configuration file.

    Attributes:
        cfg (dict): Configuration dictionary loaded from a YAML file.
        mapping (dict): Current mapping of actions based on pressed keys.
    """
    def __init__(self, cfg_file):
        """
        Initializes the InputMapper with a configuration file.
        """
        with open(cfg_file, 'r') as f:
            self.cfg = yaml.safe_load(f)
        self.mapping = {}
        self.translation = np.array([0,0,0])
        self.rotation = np.array([0,0,0])

    def axis_to_vec(self, axis: str):
        """
        Converts an axis string to a corresponding 3D vector.
        Args:
            axis (str): Axis string (e.g., "+X", "-Y", "+Z").
        Returns:
            np.ndarray: How much to move in each axis.
        """
        a = axis.upper()
        return {
            "+X": np.array([1,0,0]), "-X": np.array([-1,0,0]),
            "+Y": np.array([0,1,0]), "-Y": np.array([0,-1,0]),
            "+Z": np.array([0,0,1]), "-Z": np.array([0,0,-1]),
        }.get(a, np.array([0,0,0]))



    def yaw_pitch_roll_to_vec(self, axis: str):
        """
        Converts a yaw/pitch/roll axis string to a corresponding 3D rotation vector.
        Args:
            axis (str): Axis string (e.g., "+YAW", "-PITCH", "+ROLL").
        Returns:
            np.ndarray: How much to rotate in each axis.
        """
        a = axis.upper()
        return {
            "+YAW": np.array([0,0,1]), "-YAW": np.array([0,0,-1]),
            "+PITCH": np.array([0,1,0]), "-PITCH": np.array([0,-1,0]),
            "+ROLL": np.array([1,0,0]), "-ROLL": np.array([-1,0,0]),
        }.get(a, np.array([0,0,0]))

    def zoom_factor(self, zoom: str, zoom_factor: float):
        z = zoom.upper()
        return {
            "+ZOOM": zoom_factor,
            "-ZOOM": -zoom_factor
        }.get(z, 0)

    def initialize_mapping(self, speed, deg_s):
        """
        Initializes the mapping for a specific target from the configuration.
        Args:
            speed (float): Speed value for movement.
            deg_s (float): Degree per second value for rotation.
        Returns:
            dict: Initialized mapping dictionary for the target.
        """
        return {'speed': speed,
                'deg_s': deg_s,
                'zoom': 1,
                'local_move': np.array([0,0,0]),
                'rot_deg': np.array([0,0,0])}

    def add_mapping_per_device(self, device, keys):
        """
        generates the mapping based on pressed keys for a specific device.
        Args:
            device (str): The input device identifier.
            keys (set): Set of currently pressed keys for the device.
        """
        for key in keys:
            if key not in self.cfg['devices'][device]:
                continue
            pressed_key = self.cfg['devices'][device][key]
            if pressed_key['target'] not in self.mapping:
                self.mapping[pressed_key['target']] = self.initialize_mapping(speed=pressed_key['speed'],deg_s=pressed_key['deg_s'])
            axis_value = self.cfg['devices'][device][key]['axis']
            self.mapping[pressed_key['target']]['local_move']+=self.axis_to_vec(axis_value)
            self.mapping[pressed_key['target']]['rot_deg']+=self.yaw_pitch_roll_to_vec(axis_value)
            if 'zoom_factor' in self.cfg['profiles'][pressed_key['target']]:
                self.mapping[pressed_key['target']]['zoom']+=self.zoom_factor(axis_value, self.cfg['profiles'][pressed_key['target']]['zoom_factor'])


    def calculate_mapping(self, pressed: dict) -> dict:
        """Calculates the mapping based on currently pressed keys from all devices.
        Args:
            pressed (dict): Dictionary of pressed keys per device.
        Returns:
            dict: Mapping of targets to their movement and rotation settings.
        I/O example:
            pressed: {"keyboard1": {"W","4"}, "keyboard2": {...}}
            returns: {target_name: {'speed', 'deg_s', 'local_move', 'rot_deg'}}
        """
        self.mapping = {}
        for device, keys in pressed.items():
            try:
                self.add_mapping_per_device( device, keys)
            except:
                wakk = 2
        return self.mapping



if __name__ == "__main__":
    cfg_file  = '/home/ronim/isaacsim/source/user_scripts/Simulation/bindings.yaml'

    im = InputMapper(cfg_file)
    pressed = {
        "keyboard1": {"W","T","R","4","G","+"},
        "keyboard2": {"8","4","6","9"}
    }
    mapping = im.calculate_mapping(pressed)
    print(mapping['camera'])
    print(mapping['cube2'])
