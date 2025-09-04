

import numpy as np
from pxr import Usd, UsdGeom, Sdf, Gf
import math
import json



def euler_to_quaternion(orientation):
        yaw, pitch, roll = math.radians(orientation[0]), math.radians(orientation[1]), math.radians(orientation[2])

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

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

def get_mesh_position_from_dms(camera_position_dms, transformation):
    # find camera position from real world coordinates
    position_lla = [dms_to_dd(camera_position_dms['lon']), dms_to_dd(camera_position_dms['lat'])]
    x, y, z = lla_to_ecef(position_lla[0], position_lla[1], camera_position_dms['height'])
    transformed_vertices = np.dot(np.linalg.inv(transformation[:, :]), np.array([x, y, z, 1])).T
    C = rot_x(+90) 
    return np.dot(np.linalg.inv(C), transformed_vertices)



