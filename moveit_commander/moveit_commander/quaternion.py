#
#
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp

    return [qx, qy, qz, qw]

def quaternion_to_euler(qx, qy, qz, qw):
    n = np.linalg.norm([qx, qy, qz, qw])
    qx, qy, qz, qw = np.array([qx, qy, qz, qw])/n

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if np.fabs(sinp) >= 1:
        pitch = np.copsign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]

def as_eular_angles(qx, qy, qz, qw):
    n = np.linalg.norm([qx, qy, qz, qw])
    q = np.array([qx, qy, qz, qw])/n
    alpha = np.arctan2(q[2], q[3]) + np.arctan2(-q[0], q[1])
    beta = 2*np.arccos(np.sqrt((q[3]**2 + q[2]**2)/n))
    gamma = np.arctan2(q[2], q[3]) - np.arctan2(-q[0], q[1])
    return alpha, beta, gamma

def from_eular_angles(alpha, beta, gamma):
    qw = np.cos(beta/2.0)*np.cos((alpha+gamma)/2.0)
    qx = -np.sin(beta/2.0)*np.sin((alpha-gamma)/2.0)
    qy = np.sin(beta/2.0)*np.cos((alpha-gamma)/2.0)
    qz = np.cos(beta/2.0)*np.sin((alpha+gamma)/2.0)
    return qx, qy, qz, qw
