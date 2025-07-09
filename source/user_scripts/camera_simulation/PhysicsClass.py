

import numpy as np

class PhysicsClass():
    def __init__(self,speed = 5):
        """
        Initialize the PhysicsClass with a specified speed.
        Args:
            speed (int, optional): The speed value. Defaults to 5.
        """

        self.speed = speed



    def update_velocity_direction(self, pressed_keys):
        """
        Calculates and returns the velocity vector based on currently pressed movement keys.
        Args:
            pressed_keys (Iterable[str]): Keys currently pressed (e.g., "W", "A", "S", "D", "G", "B").
        Returns:
            np.ndarray: Normalized velocity vector scaled by self.speed.
        """


        if len(pressed_keys) == 0:
            return np.array([0.0, 0.0, 0.0])
        
        direction = np.array([0.0, 0.0, 0.0])
        if "W" in pressed_keys: direction += np.array([1.0, 0.0, 0.0])
        if "S" in pressed_keys: direction += np.array([-1.0, 0.0, 0.0])
        if "A" in pressed_keys: direction += np.array([0.0, 1.0, 0.0])
        if "D" in pressed_keys: direction += np.array([0.0, -1.0, 0.0])
        if "R" in pressed_keys: direction += np.array([0.0, 0.0, 1.0])
        if "F" in pressed_keys: direction += np.array([0.0, 0.0, -1.0])

        # Normalize direction to prevent faster diagonal movement and apply speed
        norm = np.linalg.norm(direction)
        velocity = (direction / norm) * self.speed if norm > 0 else np.array([0.0, 0.0, 0.0])
        return np.array(velocity)
    
    def update_orientation(self,pressed_keys, delta_angle = 0.5):
        """
        Updates the orientation angles based on pressed key inputs.

        Args:
            pressed_keys (set or list): Collection of currently pressed key identifiers.

        Returns:
            np.ndarray: Array containing [roll, pitch, yaw] angle changes.
        """

        roll, pitch, yaw = 0.0,0.0,0.0  # Assuming prev_angles is a numpy array with [yaw, pitch, roll]
        if "I" in pressed_keys: pitch -= delta_angle
        if "K" in pressed_keys: pitch += delta_angle
        if "J" in pressed_keys: yaw += delta_angle
        if "L" in pressed_keys: yaw -= delta_angle
        if "U" in pressed_keys: roll -= delta_angle
        if "O" in pressed_keys: roll += delta_angle
        return np.array([roll, pitch, yaw])  # Assuming roll is not needed
        
            
