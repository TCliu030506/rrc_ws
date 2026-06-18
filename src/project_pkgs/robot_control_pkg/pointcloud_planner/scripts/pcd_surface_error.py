#!/usr/bin/env python3
"""Compare a measured workpiece cloud against a predefined surface model."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def default_data_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("pointcloud_planner")) / "data"
    except Exception:
        return Path(__file__).resolve().parents[1] / "data"


def load_ascii_xyz_pcd(path: Path) -> np.ndarray:
    fields = None
    data_format = None

    with path.open("r", encoding="ascii") as pcd_file:
        for line in pcd_file:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            tokens = stripped.split()
            key = tokens[0].upper()
            if key == "FIELDS":
                fields = tokens[1:]
            elif key == "DATA":
                data_format = tokens[1].lower()
                break

        if fields is None:
            raise ValueError(f"{path}: missing FIELDS header")
        if data_format != "ascii":
            raise ValueError(
                f"{path}: only ASCII PCD is supported, got DATA {data_format}"
            )

        try:
            xyz_columns = [fields.index(axis) for axis in ("x", "y", "z")]
        except ValueError as exc:
            raise ValueError(f"{path}: FIELDS must contain x y z") from exc

        points = np.loadtxt(
            pcd_file,
            dtype=np.float64,
            usecols=xyz_columns,
            ndmin=2,
        )

    if points.shape[0] == 0:
        raise ValueError(f"{path}: contains no points")

    finite_points = points[np.isfinite(points).all(axis=1)]
    if finite_points.shape[0] == 0:
        raise ValueError(f"{path}: contains no finite XYZ points")
    return finite_points


def compute_nearest_neighbor_errors(
    model_points: np.ndarray,
    measured_points: np.ndarray,
) -> np.ndarray:
    if model_points.size == 0 or measured_points.size == 0:
        raise ValueError("point clouds must not be empty")
    distances, _ = cKDTree(model_points).query(measured_points, k=1, workers=-1)
    return np.asarray(distances, dtype=np.float64)


def compute_statistics(errors: np.ndarray) -> dict[str, float]:
    if errors.size == 0:
        raise ValueError("no errors available for statistics")
    return {
        "mean": float(np.mean(errors)),
        "rmse": float(math.sqrt(np.mean(np.square(errors)))),
        "std": float(np.std(errors)),
        "median": float(np.median(errors)),
        "p90": float(np.percentile(errors, 90)),
        "p95": float(np.percentile(errors, 95)),
        "p99": float(np.percentile(errors, 99)),
        "max": float(np.max(errors)),
    }


def write_error_csv(
    path: Path,
    measured_points: np.ndarray,
    errors: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(["x_m", "y_m", "z_m", "nearest_error_m"])
        for point, error in zip(measured_points, errors):
            writer.writerow([point[0], point[1], point[2], error])


def parse_args():
    data_dir = default_data_dir()
    parser = argparse.ArgumentParser(
        description=(
            "Compute nearest-neighbor surface error from the measured "
            "workpiece cloud to the predefined model cloud."
        )
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=data_dir / "pcl_premodel_concave_workpiece.pcd",
    )
    parser.add_argument(
        "--measured",
        type=Path,
        default=data_dir / "pcl_roi_refined_workpiece.pcd",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=0.0,
        help="Exclude distances above this value in meters; 0 disables filtering.",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        help="Optionally save each measured point and its nearest error.",
    )
    return parser.parse_known_args()


def format_statistics(stats: dict[str, float]) -> list[str]:
    labels = (
        ("mean", "mean"),
        ("rmse", "RMSE"),
        ("std", "std"),
        ("median", "median"),
        ("p90", "P90"),
        ("p95", "P95"),
        ("p99", "P99"),
        ("max", "max"),
    )
    return [
        f"{label}: {stats[key]:.8f} m ({stats[key] * 1000.0:.3f} mm)"
        for key, label in labels
    ]


def main():
    args, ros_args = parse_args()
    if args.max_distance < 0.0:
        raise ValueError("--max-distance must be >= 0")

    import rclpy

    rclpy.init(args=ros_args)
    node = rclpy.create_node("pcd_surface_error")

    try:
        node.get_logger().info(f"model: {args.model}")
        node.get_logger().info(f"measured: {args.measured}")

        model_points = load_ascii_xyz_pcd(args.model)
        measured_points = load_ascii_xyz_pcd(args.measured)
        errors = compute_nearest_neighbor_errors(model_points, measured_points)

        selected_errors = errors
        if args.max_distance > 0.0:
            selected_errors = errors[errors <= args.max_distance]
            excluded_count = errors.size - selected_errors.size
            node.get_logger().info(
                f"max_distance: {args.max_distance:.6f} m, "
                f"excluded_points: {excluded_count}"
            )
            if selected_errors.size == 0:
                raise ValueError("all measured points were excluded")

        node.get_logger().info(f"model_points: {model_points.shape[0]}")
        node.get_logger().info(f"measured_points: {measured_points.shape[0]}")
        node.get_logger().info(f"evaluated_points: {selected_errors.size}")
        for line in format_statistics(compute_statistics(selected_errors)):
            node.get_logger().info(line)

        if args.output_csv is not None:
            write_error_csv(args.output_csv, measured_points, errors)
            node.get_logger().info(f"error CSV saved: {args.output_csv}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
