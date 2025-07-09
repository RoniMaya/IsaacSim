

import numpy as np

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