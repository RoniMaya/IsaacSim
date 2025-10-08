

import numpy as np
import time
class Controller():
    """Manages the control inputs and mappings for the simulation."""

    def __init__(self, mapper):
        self.cfg = mapper['profiles']
        # self.thermal_prev = {}
        self.last_toggle = {'thermal': {}, 'slave': {}}
        self.prev_state = {'thermal': {}, 'slave': {}}


        # self.thermal_last_toggle = {}
        self.debounce_s = 0.2
        # self.slave_prev = {}
        # self.slave_last_toggle = {}

    def zoom_factor(self, mapping,key):
        return mapping[key]['zoom'] if key in mapping and 'zoom' in mapping[key] else 1

    def rot_flag(self, mapping,key):
        return mapping[key]['rot_deg'] if key in mapping and 'rot_deg' in mapping[key] else [0,0,0]
    
    def choose_camera(self, mapping):
        return mapping['asset'] if 'asset' in mapping else 'cam1'



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
    
    def toggle_camera(self, mapping,key):
        if key in mapping and 'toggle' in mapping[key]:
            return mapping[key]['toggle']



    def reset_asset(self, mapping,key):

        if key in mapping and 'reset' in mapping[key] and mapping[key]['reset'] == 1:
            return True

        return False
    

    def debounce(self, last_toggle, debounce_s):
        """Debounces a button press."""
        now = time.perf_counter()
        last = 0.0 if last_toggle is None else last_toggle
        if (now - last) >= debounce_s:
            return True, now
        return False, last_toggle


    def toggle(self,asset, mapping,asset_name,controll_function):
        pressed = mapping[asset_name][controll_function] if asset_name in mapping and controll_function in mapping[asset_name] else False # check if key is pressed
        was_pressed = self.prev_state[controll_function].get(asset_name, False) # check if key was pressed last frame
        if pressed and not was_pressed: # if key is pressed now but wasn't last frame, toggle thermal
            allowed, ts = self.debounce(self.last_toggle[controll_function].get(asset_name), self.debounce_s) # check debounce for minimum time between toggles
            if allowed: # if debounce passed, toggle thermal
                asset = not asset
                self.last_toggle[controll_function][asset_name] = ts # save last toggle time
        # Update prev for next frame
        self.prev_state[controll_function][asset_name] = pressed
        return asset



    def thermal_camera(self, thermal,mapping,key):
        pressed = mapping[key]['thermal'] if key in mapping and 'thermal' in mapping[key] else False # check if key is pressed
        was_pressed = self.thermal_prev.get(key, False) # check if key was pressed last frame
        if pressed and not was_pressed: # if key is pressed now but wasn't last frame, toggle thermal
            allowed, ts = self.debounce(self.thermal_last_toggle.get(key), self.debounce_s) # check debounce for minimum time between toggles
            if allowed: # if debounce passed, toggle thermal
                thermal = not thermal
                self.thermal_last_toggle[key] = ts # save last toggle time

        # Update prev for next frame
        self.thermal_prev[key] = pressed
        return thermal
    


    def enslave_camera(self, thermal,mapping,key):
        pressed = mapping[key]['slave'] if key in mapping and 'slave' in mapping[key] else False # check if key is pressed
        was_pressed = self.slave_prev.get(key, False) # check if key was pressed last frame
        if pressed and not was_pressed: # if key is pressed now but wasn't last frame, toggle slave
            allowed, ts = self.debounce(self.slave_last_toggle.get(key), self.debounce_s) # check debounce for minimum time between toggles
            if allowed: # if debounce passed, toggle slave
                thermal = not thermal
                self.slave_last_toggle[key] = ts # save last toggle time

        # Update prev for next frame
        self.slave_prev[key] = pressed
        return thermal


    

    def angular_velocity_p_controller(self,desired_angle_rad,current_angle_rad, asset_name, kp = 1 ):
        """ Proportional controller for angular velocity to reach a desired angle.  """
        angle_error_rad = (desired_angle_rad - current_angle_rad + np.pi) % (2 * np.pi) - np.pi
        return angle_error_rad*self.cfg[asset_name]['deg_s']*np.pi/180*kp # Use world



    def angular_velocity_pd_controller(self,desired_angle_rad,current_angle_rad,angular_velocity, asset_name, kp = 1, kd = 0.1 ):
        """ Proportional-Derivative controller for angular velocity to reach a desired angle.  """
        angle_error_rad = (desired_angle_rad - current_angle_rad + np.pi) % (2 * np.pi) - np.pi
        omega_cmd = kp * angle_error_rad - kd * angular_velocity
        max_rate = np.deg2rad(self.cfg[asset_name]['deg_s'])

        return np.clip(omega_cmd, -max_rate, max_rate)


    def get_velocity_parameters_spline_path(self,translation,current_heading_vector,spline_points,spline_points_der,asset_name, kp = 200,lookahead_idx= 8):
        """ Get velocity parameters to follow a spline path. """

        
        dist = np.linalg.norm(spline_points - translation, axis=1)
        i0 = np.argmin(np.linalg.norm(spline_points - translation, axis=1))
        i1 = min(i0 + lookahead_idx, len(spline_points) - 1)  # e.g., lookahead_idx = 8
        t_hat = spline_points_der[i1]
        direction_vector = t_hat / (np.linalg.norm(t_hat) + 1e-9)
        desired_angle_rad  = np.arctan2(t_hat[1], t_hat[0])
        current_angle_rad = np.arctan2(current_heading_vector[1], current_heading_vector[0])
        angular_velocity = self.angular_velocity_p_controller(desired_angle_rad,current_angle_rad, asset_name, kp = kp)
        return angular_velocity, direction_vector*self.cfg[asset_name]['speed']
    



