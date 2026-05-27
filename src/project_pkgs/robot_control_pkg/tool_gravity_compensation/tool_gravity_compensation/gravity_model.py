import numpy as np


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return 3x3 rotation matrix from quaternion.

    The matrix maps vectors from source frame to target frame when quaternion
    follows ROS transform convention.
    """
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n

    xx = qx * qx * s
    yy = qy * qy * s
    zz = qz * qz * s
    xy = qx * qy * s
    xz = qx * qz * s
    yz = qy * qz * s
    wx = qw * qx * s
    wy = qw * qy * s
    wz = qw * qz * s

    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def solve_gravity_params(samples):
    """Least-squares identification.

    samples: list of dict with
      - g_sensor: np.ndarray shape(3,)
      - force: np.ndarray shape(3,)
      - torque: np.ndarray shape(3,)

    Model:
      f = b_f + m * g_s
      tau = b_tau + c x g_s, where c = m * r_com

    Unknown vector x = [bfx,bfy,bfz,m,btx,bty,btz,cx,cy,cz].
    """
    n = len(samples)
    if n < 6:
        raise ValueError('Need at least 6 samples for robust estimation.')

    a = np.zeros((6 * n, 10), dtype=float)
    y = np.zeros((6 * n,), dtype=float)

    for i, s in enumerate(samples):
        gx, gy, gz = s['g_sensor']
        fx, fy, fz = s['force']
        tx, ty, tz = s['torque']
        r = 6 * i

        a[r + 0, 0] = 1.0
        a[r + 0, 3] = gx
        y[r + 0] = fx

        a[r + 1, 1] = 1.0
        a[r + 1, 3] = gy
        y[r + 1] = fy

        a[r + 2, 2] = 1.0
        a[r + 2, 3] = gz
        y[r + 2] = fz

        a[r + 3, 4] = 1.0
        a[r + 3, 8] = gz
        a[r + 3, 9] = -gy
        y[r + 3] = tx

        a[r + 4, 5] = 1.0
        a[r + 4, 7] = -gz
        a[r + 4, 9] = gx
        y[r + 4] = ty

        a[r + 5, 6] = 1.0
        a[r + 5, 7] = gy
        a[r + 5, 8] = -gx
        y[r + 5] = tz

    x, residuals, rank, _ = np.linalg.lstsq(a, y, rcond=None)

    bf = x[0:3]
    m = float(x[3])
    bt = x[4:7]
    c = x[7:10]

    if abs(m) < 1e-8:
        r_com = np.zeros(3)
    else:
        r_com = c / m

    if residuals.size == 0:
        rms = 0.0
    else:
        rms = float(np.sqrt(residuals[0] / (6 * n)))

    return {
        'force_bias': bf,
        'mass': m,
        'torque_bias': bt,
        'com': r_com,
        'c_vector': c,
        'rank': int(rank),
        'rms_residual': rms,
        'sample_count': n,
    }


def predict_gravity_wrench(g_sensor, params):
    """Compute gravity-induced force and torque from identified parameters."""
    m = float(params['mass'])
    bf = np.asarray(params['force_bias'], dtype=float)
    bt = np.asarray(params['torque_bias'], dtype=float)
    com = np.asarray(params['com'], dtype=float)
    g = np.asarray(g_sensor, dtype=float)

    force = bf + m * g
    torque = bt + np.cross(com, m * g)
    return force, torque
