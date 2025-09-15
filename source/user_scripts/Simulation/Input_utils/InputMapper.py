
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
    


    # def yaw_pitch_roll_flag(self, axis: str):
    #     """
    #     Converts a yaw/pitch/roll axis string to a corresponding 3D rotation vector.
    #     Args:
    #         axis (str): Axis string (e.g., "+YAW", "-PITCH", "+ROLL").
    #     Returns:
    #         np.ndarray: How much to rotate in each axis.
    #     """
    #     a = axis.upper()
    #     return {
    #         "+YAWF": np.array(1), "-YAWF": np.array(-1),
    #         "+PITCHF": np.array(1), "-PITCHF": np.array(-1),
    #         "+ROLLF": np.array(1), "-ROLLF": np.array(-1),
    #     }.get(a, np.array(0))

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
                'rot_deg': np.array([0,0,0]), 
                'reset': 0}

    def reset(self, reset):
        return {
            "RESET": 1,
        }.get(reset, 0)


    def apply_axis_updates(self, mapping, axis_value):
        if 'local_move' in mapping:
            mapping['local_move'] += self.axis_to_vec(axis_value)
        if 'rot_flag' in mapping:
            mapping['rot_flag'] = self.yaw_pitch_roll_to_vec(axis_value)
        if 'rot_deg' in mapping:
            mapping['rot_deg'] += self.yaw_pitch_roll_to_vec(axis_value)

        return mapping

    def apply_zoom_updates(self, mapping, zoom_value, zoom_factor):
        if 'zoom' in mapping:
            mapping['zoom'] +=self.zoom_factor(zoom_value, zoom_factor)
        return mapping

    def add_mapping_per_device(self, device, keys):
        """
        generates the mapping based on pressed keys for a specific device.
        Args:
            device (str): The input device identifier.
            keys (set): Set of currently pressed keys for the device.
        """
        input_device = self.cfg['devices'][device]
        for key in keys: # check all pressed keys
            if key not in input_device: # if the key is not mapped for this device continue
                continue

            pressed_key = input_device[key] # get the pressed key
            asset = pressed_key['target'] # get the target asset

            if asset not in self.mapping:
                self.mapping[asset] = self.initialize_mapping(speed=pressed_key['speed'],deg_s=pressed_key['deg_s'])
            
            asset_mapping = self.mapping[asset]
            asset_profile = self.cfg['profiles'][asset]

            if 'axis' in pressed_key:
                asset_mapping = self.apply_axis_updates(asset_mapping, pressed_key['axis'])
            if 'zoom_factor' in asset_profile:
                asset_mapping = self.apply_zoom_updates(asset_mapping, pressed_key['axis'], asset_profile['zoom_factor'])  
            if 'reset' in pressed_key:
                asset_mapping['reset'] = self.reset(pressed_key['reset'])




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
            self.add_mapping_per_device( device, keys)

        return self.mapping



if __name__ == "__main__":
    cfg_file  = '/home/ronim/isaacsim/source/user_scripts/Simulation/bindings.yaml'

    im = InputMapper(cfg_file)
    pressed = {
        "keyboard1": {"W","T","R","4","G","+","X"},
        "keyboard2": {"8","4","6","9"}
    }
    mapping = im.calculate_mapping(pressed)
    print(mapping['camera'])
    print(mapping['cube2'])
