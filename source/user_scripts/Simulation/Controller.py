

import numpy as np

class Controller():
    """Manages the control inputs and mappings for the simulation."""

    def __init__(self):
        pass

    def zoom_factor(self, mapping,key):
        return mapping[key]['zoom'] if key in mapping and 'zoom' in mapping[key] else 1


    def update_velocity_direction(self, mapping,key):
        """
        Updates the velocity vector based on pressed key inputs.
        Args:
            pressed_keys (set or list): Collection of currently pressed key identifiers.
        Returns:
            np.ndarray: Array containing the local velocity vector [vx, vy, vz].
        """

        if len(mapping) == 0 or key not in mapping:
            return np.array([0.0, 0.0, 0.0])

        if 'local_move' in mapping[key]:
            direction = mapping[key]['local_move']
            speed = mapping[key]['speed']
        # Normalize direction to prevent faster diagonal movement and apply speed
        norm = np.linalg.norm(direction)
        velocity = (direction / norm) * speed if norm > 0 else np.array([0.0, 0.0, 0.0])
        return np.array(velocity)

    def update_orientation(self, mapping,key):
        """
        Updates the orientation (rotation) based on pressed key inputs.
        Args:
            pressed_keys (set or list): Collection of currently pressed key identifiers.
        Returns:
            np.ndarray: Array containing the rotation in degrees [roll, pitch, yaw].
        """
        if len(mapping) == 0 or key not in mapping:
            return np.array([0.0, 0.0, 0.0])

        if key in mapping and 'rot_deg' in mapping[key]:
            direction = mapping[key]['rot_deg']
            deg_s = mapping[key]['deg_s']
        # Normalize direction to prevent faster diagonal movement and apply speed
        return np.array(direction)*deg_s  # Assuming roll is not needed
        
            
