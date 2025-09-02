import sys
sys.path.insert(0, "/home/ronim/isaacsim/third_party/pydeps")


import pickle
import numpy as np,scipy
from scipy.spatial.transform import Rotation as R
from scipy.optimize import brentq
from scipy.interpolate import RegularGridInterpolator
import Utils
import yaml
print("NumPy:", np.__version__, "SciPy:", scipy.__version__)


class Radar:

    def __init__(self, rcs_file_path, radar_prop_path, target_name, radar_origin, radar_angle=np.array([0, 90, 0]), delta_az=80.0, delta_el=40.0, det_per_sec = 1):
        self.rcs_file_path = rcs_file_path
        self.target_name = target_name
        self.radar_origin = radar_origin # defines origin of radar in world FoR
        self.radar_angle = radar_angle # defines rotation of radar
        self.radar_rotation = self.define_rotation(radar_angle) # defines rotation of radar to world FoR

        self.delta_az = delta_az
        self.delta_el = delta_el
        self.load_rcs_data()
        self.speed_of_light = 299792458.0  # m/s
        self.load_radar_properties(radar_prop_path)
        self.det_per_sec = det_per_sec
        self.time_between_detections = 1.0 / det_per_sec

    def sample_from_rcs(self):
        pt = np.array([self.total_az, self.total_el])
        return float(self._rcs_interp(pt))

    def load_rcs_data(self):
        with open(self.rcs_file_path, 'rb') as f:
            rcs_data = pickle.load(f)
        self.azimuth = rcs_data['azimuth']
        self.elevation = rcs_data['elevation']
        self.rcs_data = rcs_data['rcs']
        self._rcs_interp = RegularGridInterpolator(
        (self.azimuth, self.elevation), self.rcs_data,
        bounds_error=False, fill_value=None)

    def load_radar_properties(self, radar_prop_path):
        self.radar_properties = yaml.safe_load(open(radar_prop_path, 'r')) 
        self.radar_properties['B'] = self.speed_of_light / (2 * self.radar_properties['r_resolution'])
        self.radar_properties["wave_length"] = self.speed_of_light / self.radar_properties["fc"]  # wavelength in meters


        self.radar_properties['range_vector'] = np.arange(0, self.radar_properties['r_max'], self.radar_properties['r_resolution'])
        self.radar_properties['azimuth_vector'] = np.arange(- self.radar_properties['az_max']//2, self.radar_properties['az_max']//2, self.radar_properties['az_resolution'])
        self.radar_properties['velocity_vector'] = np.arange(0.3, self.radar_properties['vel_max'], self.radar_properties['vel_resolution'])
        self.num_of_cells = len(self.radar_properties['range_vector']) * len(self.radar_properties['azimuth_vector']) * len(self.radar_properties['velocity_vector'])
        self.radar_rotation = self.define_rotation(self.radar_angle) # rotate radar to world FoR

    def wrap180(self,a): 
        return (a + 180) % 360 - 180


    def calculate_target_in_radar_for(self, origin_world_target, target_angle, target_velocity):

        target_rotation = self.define_rotation_from_quat(target_angle) # rotate target to world FoR
        los_world = origin_world_target - self.radar_origin # LOS in world FoR
        self.target_range = np.linalg.norm(los_world)
        los_radar = self.radar_rotation.inv().apply(los_world) # rotate LOS from world FoR to radar FoR - this is the target origin in radar FoR
        u_los = los_radar / np.linalg.norm(los_radar)
        self.az_los, self.el_los = self.cartesian_to_spherical_radar_coordinates(los_radar) # azimuth and elevation of LOS in radar FoR
        
        
        target_axes_world = target_rotation.apply(np.eye(3)) # target axes in world FoR
        target_in_radar = self.radar_rotation.inv().apply(target_axes_world) # transform target to radar FoR
        target_velocity_radar = self.radar_rotation.inv().apply(target_velocity) # transform target velocity to radar FoR
        az_target, elev_target = self.cartesian_to_spherical_radar_coordinates(target_in_radar[0,:]) # azimuth and elevation of target in radar FoR
        radial_velocity = np.dot(target_velocity_radar, u_los)
        self.total_az   = self.wrap180(-az_target + self.az_los)
        self.total_el   = -elev_target + self.el_los
        self.radial_velocity = np.dot(target_velocity_radar, u_los)



    def cartesian_to_spherical_radar_coordinates(self, v):  # radar forward = +Z
        x, y, z = v
        az = np.degrees(np.arctan2(y, z))
        elevation = np.degrees(np.arctan2(x, np.hypot(z, y)))
        return self.wrap180(az), elevation

    def spherical_to_cartesian_radar_coordinates(self, azimuth, elevation, radius=1):
        radius = 1  # radius of the sphere
        elevation = np.radians(elevation)  # convert elevation to radians
        azimuth = np.radians(azimuth)  # convert azimuth to radians

        z =  radius * np.cos(elevation) * np.cos(azimuth)
        y =  radius * np.cos(elevation) * np.sin(azimuth)
        x =  radius * np.sin(elevation)
        return x, y, z

    def check_if_in_lobe(self, az_los, elev_los):
        return (az_los > -self.delta_az) & (az_los < self.delta_az) & (elev_los > -self.delta_el) & (elev_los < self.delta_el)


    def define_rotation(self, angles, rotation_type='ZYX'):
        return R.from_euler(rotation_type, angles, degrees=True)
    

    def define_rotation_from_quat(self, quat, order='xyzw'):
        """
        quat: quaternion as numpy array/list.
            order = 'xyzw' (SciPy default) or 'wxyz' depending on source.
        """
        q = np.array(quat, dtype=float)
        if order == 'wxyz':
            # convert to SciPy's xyzw
            q = np.array([q[1], q[2], q[3], q[0]])
        return R.from_quat(q)   # SciPy expects [x, y, z, w]

    def calculate_rcs(self):
        self.rcs_db, self.rcs_m2, self.rcs_swerling = 0, 0, 0
        if self.check_if_in_lobe(self.az_los, self.el_los):
            self.rcs_db = self.sample_from_rcs()
            self.rcs_m2 = 10**(self.rcs_db/10)  # convert dBsm to m^2
            self.rcs_swerling = np.random.exponential(scale=self.rcs_m2)


    def snr_linear(self, T_K=290.0, NF_lin=1.0, L_lin=1.0):
        kB = 1.380649e-23
        K = (self.radar_properties['Pt'] * self.radar_properties['Gt'] * self.radar_properties['Gt'] *
              self.radar_properties['wave_length']**2) / ((4*np.pi)**3 * kB * T_K * self.radar_properties['B'] * NF_lin * L_lin)
        self.radar_properties['snr'] = K * self.rcs_swerling / (self.target_range**4)
        self.radar_properties['K'] = K



    def calculate_snr_shnidman(self, pd, num_of_pulses,pfa , k ):
        """Calculates the required SNR using Shnidman's equation."""
        eta = np.sqrt(-0.8 * np.log(4 * pfa * (1 - pfa))) + np.sign(pd - 0.5) * np.sqrt(-0.8 * np.log(4 * pd * (1 - pd)))
        alpha = 0 if num_of_pulses < 40 else 1/4
        x_inf = eta * (eta + 2 * np.sqrt(num_of_pulses / 2 + (alpha - 1 / 4)))
        c1 = (((17.7006 * pd - 18.4496) * pd + 14.5339) * pd - 3.525) / k
        c2 = (1 / k) * (np.exp(27.31 * pd - 25.14) + (pd - 0.8) * (0.7 * np.log(10**-5 / pfa + (2 * num_of_pulses - 20) / 80)))
        cdb = c1 if pd > 0.872 else c1 + c2
        c = 10**(cdb / 10)
        snr_linear = c * x_inf / num_of_pulses
        return snr_linear


    def objective_function(self,pd, target_snr, num_pulses, pfa_val, k_val):
        return self.calculate_snr_shnidman(pd, num_pulses, pfa_val, k_val) - target_snr

    def check_snr_range(self,num_of_pulses, k,pd_min=0.1, pd_max=0.99):
        snr_pd_min = self.calculate_snr_shnidman(pd_min, num_of_pulses, self.radar_properties['pfa'], k)
        snr_pd_max = self.calculate_snr_shnidman(pd_max, num_of_pulses, self.radar_properties['pfa'], k)
        if self.radar_properties['snr'] < snr_pd_min:
            return 1e-6
        elif self.radar_properties['snr'] > snr_pd_max:
            return 1-1e-6


    def generate_noise(self,snr, resolution):
        """
        Generate noise based on the SNR and range resolution.
        """
        sigma_r = resolution / (np.sqrt(2 * max(snr, 1e-6)))  # noise standard deviation for detection
        return np.random.normal(0, sigma_r)  # Generate normal noise



    def calculate_pd(self, num_of_pulses, k):
        """Return Pd given self.radar_properties['snr'] (actual), using Shnidman.
        Handles too-low / too-high SNR by clamping to near 0/1."""
        snr = self.radar_properties['snr']
        self.radar_properties['pd'] = self.check_snr_range( num_of_pulses, k)
        if self.radar_properties['pd'] is None:
            self.radar_properties['pd'] = brentq(self.objective_function, a=0.1, b=0.99, args=(self.radar_properties['snr'],num_of_pulses, self.radar_properties['pfa'], k))


    


    def generate_false_alarms(self, num_of_pulses):
        false_alarm = {}
        # theoreticaly we can generate a noise map with the size of the detection map 
        # and use the TH to get the false identification but its computationaly intensive
        # so we will generate random indices for the false alarms range, azimuth and velocity cells
        num_of_expected_false_alarms = self.num_of_cells * self.radar_properties['pfa']  # Expected number of false alarms
        num_false_alarms = np.random.poisson(num_of_expected_false_alarms) # Given an average rate of occurrence, how many events are likely to happen in this interval
        velocity,range_detection,azimuth = self.radar_properties['velocity_vector'], self.radar_properties['range_vector'], self.radar_properties['azimuth_vector']
        # get random indices for the false alarms range, azimuth and velocity cells
        false_alarm['velocity'] = (np.random.uniform(velocity[0], velocity[-1], size=num_false_alarms)).tolist()
        false_alarm['range'] = (np.random.uniform(range_detection[0], range_detection[-1], size=num_false_alarms)).tolist()
        false_alarm['azimuth'] = (np.random.uniform(azimuth[0], azimuth[-1], size=num_false_alarms)).tolist()
        return false_alarm





    def generate_targets(self):
        target_detection = {'range': [], 'azimuth': [], 'velocity': []}
         # decide if the target is detected based on the Pd
        snr,r_resolution, az_resolution, vel_resolution = self.radar_properties['snr'], self.radar_properties['r_resolution'], self.radar_properties['az_resolution'], self.radar_properties['vel_resolution']
        if np.random.binomial(n=1, p=self.radar_properties['pd'], size=1) == 1:
            target_detection['range'] = [self.target_range + self.generate_noise(snr, r_resolution)]  # Add noise to the target range
            target_detection['azimuth'] = [self.az_los + self.generate_noise(snr, az_resolution)]  # Add noise to the target angle
            target_detection['velocity'] = [self.radial_velocity + self.generate_noise(snr, vel_resolution)]  # Add noise to the target velocity
        return target_detection
       

    def get_detections(self, origin_world_target, target_angle, target_velocity):
        self.calculate_target_in_radar_for(origin_world_target, target_angle, target_velocity)
        self.calculate_rcs()
        self.snr_linear()
        self.calculate_pd( num_of_pulses=5, k=1)
        false_alarm = self.generate_false_alarms(num_of_pulses=5)
        target = self.generate_targets()
        return target, false_alarm


    def print_detections(self, text_for_image,target, false_alarm, passed_time):
        for key in ["range", "azimuth", "velocity"]:
            false_alarm_text = Utils.text_from_data(false_alarm, key, decimals=2)
            target_text = Utils.text_from_data(target, key, decimals=2)
            text_for_image[key] = f'{key.capitalize()} : {target_text + false_alarm_text} '
        text_for_image['Time'] = f'Time : {round(passed_time,2)} s'
        return '\n'.join(list(text_for_image.values()))
    




if __name__ == "__main__":

    print("Radar simulation test")
    print("initilizing radar")
    rcs_file_path = '/home/ronim/Documents/radar_sim/radar_rcs_maps/rcs_ball_1m_1.pkl'
    radar_prop_path = '/home/ronim/Documents/radar_sim/radar_parameters/MAGOS.yaml'

    delta_az = 80
    delta_el = 30
    radar_angle = [0, 90, 0]               
    origin_world_radar = np.array([0, 0, 3])
    radar = Radar(rcs_file_path, radar_prop_path, "ball", origin_world_radar, radar_angle, delta_az=delta_az, delta_el=delta_el)

    print("this will be the simulation loop")
    print("running with a target moving at 10m/s in x direction")
    target_velocity = np.array([10, 0, 0])  # target velocity in m/s in world FoR
    target_angle = [0, 0, 0]
    origin_world_target = np.array([50, 3, 10])

    import time
    start_time = time.time()
    # this is a time tick for the simulation loop:
    radar.calculate_target_in_radar_for(origin_world_target, target_angle, target_velocity)
    radar.calculate_rcs()
    radar.snr_linear()
    radar.calculate_pd( num_of_pulses=5, k=1)
    radar.generate_false_alarms(num_of_pulses=5)
    radar.generate_targets()
    time_elapsed = time.time() - start_time
    print(f"calculation time {time_elapsed*1000:.2f} ms, 90 fps is {1/90*1000:.2f} ms")

    start_time = time.time()
    radar.get_detections(origin_world_target, target_angle, target_velocity)
    time_elapsed = time.time() - start_time
    print(f"calculation time {time_elapsed*1000:.2f} ms, 90 fps is {1/90*1000:.2f} ms")
