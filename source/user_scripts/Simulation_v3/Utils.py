

import numpy as np
from pxr import Usd, UsdGeom, Sdf, Gf
import math
import json
from scipy.interpolate import splprep, splev
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from PIL import Image, ImageDraw, ImageFont



def euler_to_quaternion(orientation):
        yaw, pitch, roll = math.radians(orientation[0]), math.radians(orientation[1]), math.radians(orientation[2])

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qw,qx, qy, qz]

def open_coords_file(COORDINATES):
    with open(COORDINATES, "r") as f:
        lines = f.read().strip().splitlines()
    return {
        "datum": lines[0].split(' ')[0],
        "projection": lines[0].split(' ')[1],
        "zone_num": int(lines[0].split(' ')[2][:-1]),
        "zone_letter": lines[0].split(' ')[2][-1],
        "lat": float(lines[1].split(' ')[0]),
        "lon": float(lines[1].split(' ')[1])
    }


def quat_rotate_numpy(q, v):
    """
    Rotates a vector v by a quaternion q.

    Args:
        q (np.array): A quaternion in (w, x, y, z) format.
        v (np.array): A 3D vector.

    Returns:
        np.array: The rotated 3D vector.
    """
    q_w = q[0]
    q_vec = q[1:]
    
    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * 2.0 * q_w
    c = q_vec * np.dot(q_vec, v) * 2.0
    
    return a + b + c



def text_from_data(data,key, decimals = 2):
    if data is None or key not in data:        
        return []
    else:
        text_in_list = [f"{x:.{decimals}f}" for x in data[key]]
        return text_in_list
    

def check_time_for_action(now, next_action, frequency):
    if now >= next_action:
        next_action += 1.0 / frequency
        return True, next_action
    return False, next_action


def get_usd_props(asset_path):
    sub = Usd.Stage.Open(asset_path)
    mpu = UsdGeom.GetStageMetersPerUnit(sub)
    up = UsdGeom.GetStageUpAxis(sub)
    return mpu, up



def ecef_to_lla(x, y, z):
    # WGS84 constants
    a  = 6378137.0
    e2 = 6.6943799901413165e-3
    b  = a * math.sqrt(1 - e2)
    ep2 = (a*a - b*b) / (b*b)

    lon = math.atan2(y, x)
    p   = math.hypot(x, y)
    theta = math.atan2(z * a, p * b)
    st, ct = math.sin(theta), math.cos(theta)

    lat = math.atan2(z + ep2 * b * st**3, p - e2 * a * ct**3)
    N   = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    h   = p / math.cos(lat) - N

    return math.degrees(lon), math.degrees(lat), h


def lla_to_ecef(lon, lat, h):
    # WGS84 constants
    a  = 6378137.0
    e2 = 6.6943799901413165e-3

    lon = math.radians(lon)
    lat = math.radians(lat)

    N = a / math.sqrt(1 - e2 * math.sin(lat)**2)

    x = (N + h) * math.cos(lat) * math.cos(lon)
    y = (N + h) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - e2) + h) * math.sin(lat)

    return x, y, z



def dms_to_dd(dmsd):
    dd = float(dmsd[0]) + float(dmsd[1])/60 + float(dmsd[2])/(60*60)
    if dmsd[3] in ['S', 'W']:
        dd *= -1
    return dd



def dd_to_dms(dd):
    dd = float(dd)
    degrees = int(dd)
    minutes = int((dd - degrees) * 60)
    seconds = (dd - degrees - minutes / 60) * 3600
    return [degrees, minutes, seconds]




def rot_x(deg = 90):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[1,0,0,0],
                     [0,c,-s,0],
                     [0,s, c,0],
                     [0,0,0,1]], dtype=float)


def rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([
        [ c,-s, 0, 0],
        [ s, c, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]], float)

def cesium_transformation(cesium_conf_path):
    with open(cesium_conf_path, 'r') as f:
        config = json.load(f)
    return np.array(config['root']['transform']).reshape(4, 4, order='F')


def get_mesh_position_from_dms(camera_position_dms, transformation,lla = False):
    # find camera position from real world coordinates
    if lla == False:
        position_lla = [dms_to_dd(camera_position_dms['lon']), dms_to_dd(camera_position_dms['lat'])]
    else:
        position_lla = [camera_position_dms['lon'], camera_position_dms['lat']]
    x, y, z = lla_to_ecef(position_lla[0], position_lla[1], camera_position_dms['height'])
    transformed_vertices = np.dot(np.linalg.inv(transformation[:, :]), np.array([x, y, z, 1])).T
    C = rot_x(+90) 
    return np.dot(np.linalg.inv(C), transformed_vertices)


def generate_spline_path(spline_position_dms, cesium_transformation, spline_param = 3, num_samples = 500, add_z = 0):
    waypoints = np.vstack([get_mesh_position_from_dms(dms_position, cesium_transformation) for dms_position in spline_position_dms])
    tck, u = splprep([waypoints[:,0], waypoints[:,1], waypoints[:,2]], s=0, k=spline_param)
    spline_points_der = np.array(splev(np.linspace(0, 1, num_samples), tck, der = 1)).T
    spline_points = np.array(splev(np.linspace(0, 1, num_samples), tck)).T + np.array([0,0,add_z])
    ini_dir = (spline_points_der[1] - spline_points_der[0])/np.linalg.norm(spline_points_der[1] - spline_points_der[0])
    euler_initial_angles = np.arctan2(ini_dir[0], ini_dir[1])*180/np.pi
    return spline_points,spline_points_der, euler_initial_angles


def generate_spline_path_from_enu(waypoints, spline_param = 3, num_samples = 500, add_z = 0):
    tck, u = splprep([waypoints[:,0], waypoints[:,1], waypoints[:,2]], s=0, k=spline_param)
    spline_points_der = np.array(splev(np.linspace(0, 1, num_samples), tck, der = 1)).T
    spline_points = np.array(splev(np.linspace(0, 1, num_samples), tck)).T + np.array([0,0,add_z])
    ini_dir = (spline_points[1] - spline_points[0])/np.linalg.norm(spline_points[1] - spline_points[0])
    euler_initial_angles = np.arctan2(ini_dir[1], ini_dir[0])*180/np.pi
    return spline_points,spline_points_der, euler_initial_angles





def _make_polar_rgba(az_deg, r_m, snr=None, r_max=None, size=(240, 240)):
    """
    Return an RGBA numpy array of a polar plot (theta=azimuth, r=range).
    - az_deg, r_m: 1D arrays
    - snr: optional 1D array for color mapping
    - r_max: optional float to fix radial limit
    - size: (width, height) in pixels
    """
    w, h = size
    dpi = 100
    fig = Figure(figsize=(w/dpi, h/dpi), dpi=dpi)
    fig.patch.set_alpha(0)  # transparent figure
    ax = fig.add_subplot(111, projection='polar')
    ax.set_facecolor('none')

    theta = np.deg2rad(np.asarray(az_deg))
    r = np.asarray(r_m)

    if snr is None:
        ax.scatter(theta, r, s=10, alpha=0.95, color = 'red')  # default color
    else:
        sc = ax.scatter(theta, r, c=np.asarray(snr), s=12, alpha=0.95, cmap='viridis')
        # Optional: a colorbar would take space; usually skip for an inset

    if r_max is not None:
        ax.set_rlim(0, r_max)

    # Clean up for inset look
    ax.grid(True, alpha=0.3)
    ax.set_xticklabels([])         # hide az labels
    ax.set_yticklabels([])         # hide r labels
    ax.spines['polar'].set_visible(False)

    canvas = FigureCanvas(fig)
    canvas.draw()
    rgba = np.asarray(canvas.buffer_rgba())  # HxWx4 (RGBA)
    return rgba




def overlay_polar_inset(frame_rgba, az_deg, r_m, snr=None, r_max=300, pos=(20, 20), size=(240, 240)):
    """
    Paste the polar plot onto frame_rgba at pixel 'pos' (x,y).
    Returns a new numpy array (same shape as frame_rgba).
    """
    plot_rgba = _make_polar_rgba(az_deg, r_m, snr=snr, r_max=r_max, size=size)
    base = Image.fromarray(frame_rgba) if not isinstance(frame_rgba, Image.Image) else frame_rgba
    inset = Image.fromarray(plot_rgba, mode="RGBA")
    base.paste(inset, pos, mask=inset)  # alpha composite
    return np.asarray(base)




