import numpy as np


_LINK_COUNT = 4
_PARAMETER_COUNT = 16


def _skew(vector):
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=float)


def _positive_scalar(value, name):
    scalar = float(value)
    if not np.isfinite(scalar) or scalar <= 0.0:
        raise ValueError(f"{name} must be finite and > 0")
    return scalar


def _finite_array(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _constraint_parameterization(base_mass, moving_total_mass):
    constraints = np.zeros((2, _PARAMETER_COUNT), dtype=float)
    constraints[0, 0] = 1.0
    constraints[1, 1:4] = 1.0
    target = np.array([base_mass, moving_total_mass], dtype=float)

    left, singular_values, right_t = np.linalg.svd(
        constraints, full_matrices=True
    )
    tolerance = (
        max(constraints.shape)
        * np.finfo(float).eps
        * singular_values[0]
    )
    rank = int(np.sum(singular_values > tolerance))
    particular = (
        right_t[:rank].T
        @ ((left[:, :rank].T @ target) / singular_values[:rank])
    )
    null_space = right_t[rank:].T
    return constraints, target, particular, null_space


def _regularization_system(
    base_mass,
    moving_mass_prior,
    moving_mass_prior_std_kg,
    com_xy_prior_std_m,
    first_moment_min_norm_std_kg_m,
):
    rows = []
    targets = []

    for moving_index, prior_mass in enumerate(moving_mass_prior, start=1):
        row = np.zeros(_PARAMETER_COUNT, dtype=float)
        row[moving_index] = 1.0 / moving_mass_prior_std_kg
        rows.append(row)
        targets.append(prior_mass / moving_mass_prior_std_kg)

    reference_masses = np.concatenate(([base_mass], moving_mass_prior))
    for link_index, reference_mass in enumerate(reference_masses):
        xy_std = reference_mass * com_xy_prior_std_m
        moment_start = 4 + 3 * link_index
        for component in (0, 1):
            row = np.zeros(_PARAMETER_COUNT, dtype=float)
            row[moment_start + component] = 1.0 / xy_std
            rows.append(row)
            targets.append(0.0)

    for moment_index in range(4, _PARAMETER_COUNT):
        row = np.zeros(_PARAMETER_COUNT, dtype=float)
        row[moment_index] = 1.0 / first_moment_min_norm_std_kg_m
        rows.append(row)
        targets.append(0.0)

    return np.asarray(rows), np.asarray(targets)


def solve_constrained_dynamic_gravity_params(
    samples,
    base_mass,
    moving_total_mass,
    force_bias,
    torque_bias,
    com_xy_prior_std_m,
    moving_mass_prior,
    moving_mass_prior_std_kg,
    first_moment_min_norm_std_kg_m,
    min_link_mass_kg,
):
    base_mass = _positive_scalar(base_mass, "base_mass")
    moving_total_mass = _positive_scalar(
        moving_total_mass, "moving_total_mass"
    )
    com_xy_prior_std_m = _positive_scalar(
        com_xy_prior_std_m, "com_xy_prior_std_m"
    )
    moving_mass_prior_std_kg = _positive_scalar(
        moving_mass_prior_std_kg, "moving_mass_prior_std_kg"
    )
    first_moment_min_norm_std_kg_m = _positive_scalar(
        first_moment_min_norm_std_kg_m,
        "first_moment_min_norm_std_kg_m",
    )
    min_link_mass_kg = _positive_scalar(
        min_link_mass_kg, "min_link_mass_kg"
    )
    force_bias = _finite_array(force_bias, (3,), "force_bias")
    torque_bias = _finite_array(torque_bias, (3,), "torque_bias")
    moving_mass_prior = _finite_array(
        moving_mass_prior, (3,), "moving_mass_prior"
    )
    if np.any(moving_mass_prior <= 0.0):
        raise ValueError("moving_mass_prior values must be > 0")
    if abs(float(np.sum(moving_mass_prior)) - moving_total_mass) > 1e-9:
        raise ValueError(
            "moving_mass_prior sum must equal moving_total_mass within 1e-9"
        )
    if base_mass < min_link_mass_kg:
        raise ValueError("base_mass must be >= min_link_mass_kg")
    if moving_total_mass < (_LINK_COUNT - 1) * min_link_mass_kg:
        raise ValueError(
            "moving_total_mass cannot satisfy min_link_mass_kg"
        )

    samples = list(samples)
    if not samples:
        raise ValueError("at least one sample is required")

    system = np.zeros((6 * len(samples), _PARAMETER_COUNT), dtype=float)
    target = np.zeros(6 * len(samples), dtype=float)
    for sample_index, sample in enumerate(samples):
        g_sensor = _finite_array(
            sample["g_sensor"], (3,), f"samples[{sample_index}].g_sensor"
        )
        force = _finite_array(
            sample["force"], (3,), f"samples[{sample_index}].force"
        )
        torque = _finite_array(
            sample["torque"], (3,), f"samples[{sample_index}].torque"
        )
        transforms = list(sample["transforms"])
        if len(transforms) != _LINK_COUNT:
            raise ValueError(
                "sample transform count must be 4, "
                f"got {len(transforms)}"
            )

        row = 6 * sample_index
        force_rows = slice(row, row + 3)
        torque_rows = slice(row + 3, row + 6)
        target[force_rows] = force - force_bias
        target[torque_rows] = torque - torque_bias
        negative_skew_g = -_skew(g_sensor)

        for link_index, transform in enumerate(transforms):
            rotation = _finite_array(
                transform["rot"],
                (3, 3),
                f"samples[{sample_index}].transforms[{link_index}].rot",
            )
            translation = _finite_array(
                transform["trans"],
                (3,),
                f"samples[{sample_index}].transforms[{link_index}].trans",
            )
            system[force_rows, link_index] = g_sensor
            system[torque_rows, link_index] = _skew(translation) @ g_sensor
            moment_start = 4 + 3 * link_index
            system[
                torque_rows, moment_start:moment_start + 3
            ] = negative_skew_g @ rotation

    constraints, constraint_target, particular, null_space = (
        _constraint_parameterization(base_mass, moving_total_mass)
    )
    regularization, regularization_target = _regularization_system(
        base_mass,
        moving_mass_prior,
        moving_mass_prior_std_kg,
        com_xy_prior_std_m,
        first_moment_min_norm_std_kg_m,
    )
    reduced_data = system @ null_space
    augmented = np.vstack((
        reduced_data,
        regularization @ null_space,
    ))
    augmented_target = np.concatenate((
        target - system @ particular,
        regularization_target - regularization @ particular,
    ))
    free_solution, _, augmented_rank, singular_values = np.linalg.lstsq(
        augmented, augmented_target, rcond=None
    )
    solution = particular + null_space @ free_solution

    masses = solution[:4]
    if np.any(masses < min_link_mass_kg):
        raise ValueError(
            "solved link mass is below min_link_mass_kg"
        )
    mass_constraint_error = float(
        np.max(np.abs(constraints @ solution - constraint_target))
    )
    if mass_constraint_error > 1e-9:
        raise ValueError("hard mass constraint error exceeds 1e-9")

    physical_residual = system @ solution - target
    rms_residual = float(
        np.sqrt(np.mean(physical_residual * physical_residual))
    )
    first_moments = solution[4:].reshape(_LINK_COUNT, 3)
    coms = first_moments / masses[:, None]
    if singular_values.size == 0 or singular_values[-1] == 0.0:
        condition_number = float("inf")
    else:
        condition_number = float(
            singular_values[0] / singular_values[-1]
        )

    return {
        "force_bias": force_bias.copy(),
        "torque_bias": torque_bias.copy(),
        "link_masses": masses,
        "link_first_moments": first_moments,
        "link_coms": coms,
        "sample_count": int(len(samples)),
        "data_rank": int(np.linalg.matrix_rank(system)),
        "reduced_data_rank": int(np.linalg.matrix_rank(reduced_data)),
        "augmented_rank": int(augmented_rank),
        "rms_residual": rms_residual,
        "mass_constraint_error": mass_constraint_error,
        "condition_number": condition_number,
    }
