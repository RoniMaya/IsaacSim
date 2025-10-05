import yaml
import numpy as np

class InputMapper():
    """
    InputMapper maps input device keys to actions based on a configuration file.

    Attributes:
        cfg (dict): Configuration dictionary loaded from a YAML file.
        mapping (dict): The final mapping generated for the current frame.
        active_camera_target (str): The name of the currently active camera profile.
    """
    def __init__(self, cfg_file):
        """
        Initializes the InputMapper with a configuration file.
        """
        with open(cfg_file, 'r') as f:
            self.cfg = yaml.safe_load(f)
        self.mapping = {}
        # Set the default active camera from the config, or fallback to a default name.
        self.active_camera_target = self.cfg.get('default_active_camera', 'cam1')

    def axis_to_vec(self, axis: str) -> np.ndarray:
        """Converts an axis string to a corresponding 3D vector."""
        a = axis.upper()
        return {
            "+X": np.array([1, 0, 0]), "-X": np.array([-1, 0, 0]),
            "+Y": np.array([0, 1, 0]), "-Y": np.array([0, -1, 0]),
            "+Z": np.array([0, 0, 1]), "-Z": np.array([0, 0, -1]),
        }.get(a, np.array([0, 0, 0]))

    def yaw_pitch_roll_to_vec(self, axis: str) -> np.ndarray:
        """Converts a yaw/pitch/roll axis string to a corresponding 3D rotation vector."""
        a = axis.upper()
        return {
            "+YAW": np.array([0, 0, 1]), "-YAW": np.array([0, 0, -1]),
            "+PITCH": np.array([0, 1, 0]), "-PITCH": np.array([0, -1, 0]),
            "+ROLL": np.array([1, 0, 0]), "-ROLL": np.array([-1, 0, 0]),
        }.get(a, np.array([0, 0, 0]))

    def zoom_factor(self, zoom: str, zoom_factor: float) -> float:
        """Calculates the zoom adjustment."""
        z = zoom.upper()
        return {
            "+ZOOM": zoom_factor,
            "-ZOOM": -zoom_factor
        }.get(z, 0)
        
    def reset_value(self, reset_cmd: str) -> int:
        """Returns 1 if the command is RESET."""
        return 1 if reset_cmd.upper() == "RESET" else 0
    

    def thermal_value(self, thermal_cmd: str) -> int:
        """Returns 1 if the command is THERMAL."""
        return True if thermal_cmd.upper() == "THERMAL" else False

    def slave_value(self, slave_cmd: str) -> int:
        """Returns 1 if the command is SLAVE."""
        return True if slave_cmd.upper() == "SLAVE" else False


    def initialize_mapping(self, speed: float, deg_s: float) -> dict:
        """
        Initializes the mapping dictionary for a target.
        """
        return {
            'speed': speed,
            'deg_s': deg_s,
            'zoom': 1.0,
            'local_move': np.array([0, 0, 0]),
            'rot_deg': np.array([0, 0, 0]),
            'reset': 0,
        }

    def apply_axis_updates(self, mapping: dict, axis_value: str):
        """Applies movement and rotation updates to a mapping."""
        mapping['local_move'] += self.axis_to_vec(axis_value)
        mapping['rot_deg'] += self.yaw_pitch_roll_to_vec(axis_value)

    def apply_zoom_updates(self, mapping: dict, zoom_value: str, zoom_factor: float):
        """Applies zoom updates to a mapping."""
        mapping['zoom'] += self.zoom_factor(zoom_value, zoom_factor)

    def calculate_mapping(self, pressed: dict) -> dict:
        """
        Calculates the final mapping based on all currently pressed keys.

        This function works in two phases:
        1. It iterates through all keys to find 'toggle' commands, updating the
           internal state for the active camera (`self.active_camera_target`).
        2. It iterates through all keys again to process actions (move, zoom, etc.),
           applying them to the correct target based on the updated state.
        
        Args:
            pressed (dict): Dictionary of pressed keys per device. 
                            Example: {"keyboard1": {"W", "2"}}
        Returns:
            dict: Mapping of targets to their movement and rotation settings.
        """
        self.mapping = {}

        # --- Phase 1: Process Toggles to Update State ---
        for device, keys in pressed.items():
            device_cfg = self.cfg['devices'].get(device, {})
            for key in keys:
                key_action = device_cfg.get(key)
                if key_action and 'toggle' in key_action:
                    # Update the active camera based on the toggle command
                    self.active_camera_target = key_action['toggle']

        # --- Phase 2: Process Actions ---
        for device, keys in pressed.items():
            device_cfg = self.cfg['devices'].get(device, {})
            for key in keys:
                key_action = device_cfg.get(key)
                # Skip if key is not defined or is a toggle key (already handled)
                if not key_action or 'toggle' in key_action:
                    continue

                # Determine the actual target for this action
                target_name = key_action['target']
                if target_name == 'camera_active':
                    # If the target is the generic 'camera_active', use the state we determined in phase 1
                    actual_target = self.active_camera_target
                else:
                    # Otherwise, use the specific target defined (e.g., a drone)
                    actual_target = target_name
                
                # Get the profile for the actual target
                profile = self.cfg['profiles'].get(actual_target)
                if not profile:
                    continue # Skip if the target has no defined profile

                # If this is the first action for this target, initialize its mapping
                if actual_target not in self.mapping:
                    self.mapping[actual_target] = self.initialize_mapping(
                        speed=profile.get('speed', 1.0),
                        deg_s=profile.get('deg_s', 90.0)
                    )

                # Apply the specific action (axis, zoom, reset) to the target's mapping
                current_mapping = self.mapping[actual_target]
                if 'axis' in key_action:
                    self.apply_axis_updates(current_mapping, key_action['axis'])
                if 'zoom' in key_action:
                    self.apply_zoom_updates(current_mapping, key_action['zoom'], profile.get('zoom_factor', 0.1))
                if 'reset' in key_action:
                    current_mapping['reset'] = self.reset_value(key_action['reset'])
                if 'thermal' in key_action:
                    current_mapping['thermal'] = self.thermal_value(key_action['thermal'])
                if 'slave' in key_action:
                    current_mapping['slave'] = self.slave_value(key_action['slave'])

        return self.mapping