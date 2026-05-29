import numpy as np


def skew(vec) -> np.ndarray:
    x, y, z = np.asarray(vec, dtype=float)
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=float)


def _as_vector3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,):
        raise ValueError(f'{name} must have shape (3,), got {array.shape}')
    return array


def _as_matrix3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3):
        raise ValueError(f'{name} must have shape (3, 3), got {array.shape}')
    return array


def _sample_transforms(sample, link_count: int) -> list[dict]:
    transforms = list(sample['transforms'])
    if len(transforms) != link_count:
        raise ValueError(
            f'sample transform count must be {link_count}, got {len(transforms)}'
        )
    return transforms


def solve_dynamic_gravity_params(samples, link_count: int) -> dict:
    if link_count <= 0:
        raise ValueError('link_count must be > 0')
    if not samples:
        raise ValueError('at least one sample is required')

    unknown_count = 6 + 4 * link_count
    system = np.zeros((6 * len(samples), unknown_count), dtype=float)
    target = np.zeros((6 * len(samples),), dtype=float)

    force_bias_start = 0
    torque_bias_start = 3
    mass_start = 6
    first_moment_start = mass_start + link_count

    for sample_index, sample in enumerate(samples):
        g_sensor = _as_vector3(sample['g_sensor'], 'g_sensor')
        force = _as_vector3(sample['force'], 'force')
        torque = _as_vector3(sample['torque'], 'torque')
        transforms = _sample_transforms(sample, link_count)

        row = 6 * sample_index
        force_rows = slice(row, row + 3)
        torque_rows = slice(row + 3, row + 6)

        target[force_rows] = force
        target[torque_rows] = torque
        system[force_rows, force_bias_start:force_bias_start + 3] = np.eye(3)
        system[torque_rows, torque_bias_start:torque_bias_start + 3] = np.eye(3)

        neg_skew_g = -skew(g_sensor)
        for link_index, transform in enumerate(transforms):
            rot = _as_matrix3(transform['rot'], f'transforms[{link_index}].rot')
            trans = _as_vector3(transform['trans'], f'transforms[{link_index}].trans')

            mass_col = mass_start + link_index
            system[force_rows, mass_col] = g_sensor
            system[torque_rows, mass_col] = skew(trans) @ g_sensor

            moment_col = first_moment_start + 3 * link_index
            system[torque_rows, moment_col:moment_col + 3] = neg_skew_g @ rot

    solution, _, rank, _ = np.linalg.lstsq(system, target, rcond=None)
    residual = system @ solution - target
    rms_residual = float(np.sqrt(np.mean(residual * residual)))

    force_bias = solution[force_bias_start:force_bias_start + 3]
    torque_bias = solution[torque_bias_start:torque_bias_start + 3]
    masses = solution[mass_start:mass_start + link_count]
    first_moments = solution[first_moment_start:].reshape(link_count, 3)
    coms = np.zeros((link_count, 3), dtype=float)
    for index, mass in enumerate(masses):
        if abs(mass) > 1e-12:
            coms[index] = first_moments[index] / mass

    return {
        'force_bias': force_bias,
        'torque_bias': torque_bias,
        'link_masses': masses,
        'link_first_moments': first_moments,
        'link_coms': coms,
        'rank': int(rank),
        'rms_residual': rms_residual,
        'sample_count': int(len(samples)),
    }


def predict_dynamic_gravity_wrench(g_sensor, transforms, params) -> tuple[np.ndarray, np.ndarray]:
    g = _as_vector3(g_sensor, 'g_sensor')
    link_masses = [float(value) for value in params['link_masses']]
    link_coms = [np.asarray(value, dtype=float) for value in params['link_coms']]
    if len(transforms) != len(link_masses) or len(link_masses) != len(link_coms):
        raise ValueError('transforms, link_masses, and link_coms lengths must match')

    force_model = np.asarray(params.get('force_bias', [0.0, 0.0, 0.0]), dtype=float).copy()
    torque_model = np.asarray(params.get('torque_bias', [0.0, 0.0, 0.0]), dtype=float).copy()
    if force_model.shape != (3,) or torque_model.shape != (3,):
        raise ValueError('force_bias and torque_bias must have shape (3,)')

    for index, (transform, mass, com_local) in enumerate(zip(transforms, link_masses, link_coms)):
        rot = _as_matrix3(transform['rot'], f'transforms[{index}].rot')
        trans = _as_vector3(transform['trans'], f'transforms[{index}].trans')
        com = _as_vector3(com_local, f'link_coms[{index}]')
        com_sensor = trans + rot @ com
        force_i = mass * g
        torque_i = np.cross(com_sensor, force_i)
        force_model += force_i
        torque_model += torque_i

    return force_model, torque_model


def aggregate_rigid_body_params(link_masses, link_coms, transforms) -> tuple[float, np.ndarray]:
    masses = [float(value) for value in link_masses]
    coms = [np.asarray(value, dtype=float) for value in link_coms]
    if len(transforms) != len(masses) or len(masses) != len(coms):
        raise ValueError('transforms, link_masses, and link_coms lengths must match')

    total_mass = float(sum(masses))
    if abs(total_mass) <= 1e-12:
        return 0.0, np.zeros(3, dtype=float)

    weighted_com = np.zeros(3, dtype=float)
    for index, (transform, mass, com_local) in enumerate(zip(transforms, masses, coms)):
        rot = _as_matrix3(transform['rot'], f'transforms[{index}].rot')
        trans = _as_vector3(transform['trans'], f'transforms[{index}].trans')
        com = _as_vector3(com_local, f'link_coms[{index}]')
        com_sensor = trans + rot @ com
        weighted_com += mass * com_sensor

    return total_mass, weighted_com / total_mass


def predict_rigid_body_inertia_wrench(
    total_mass,
    com_sensor,
    linear_accel_sensor,
) -> tuple[np.ndarray, np.ndarray]:
    mass = float(total_mass)
    com = _as_vector3(com_sensor, 'com_sensor')
    linear_accel = _as_vector3(linear_accel_sensor, 'linear_accel_sensor')

    force_model = -mass * linear_accel
    torque_model = np.cross(com, force_model)
    return force_model, torque_model
