#!/usr/bin/env python3
"""Convert the first three columns of a path file to an ASCII PCD file."""

import argparse
from pathlib import Path


def default_input_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "data"
        / "path_planning"
        / "path_map_concave.txt"
    )


def load_xyz_points(path: Path) -> list[tuple[str, str, str]]:
    points = []
    with path.open("r", encoding="ascii") as input_file:
        for line_number, line in enumerate(input_file, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            columns = stripped.split()
            if len(columns) < 3:
                raise ValueError(
                    f"{path}:{line_number}: expected at least 3 columns"
                )
            try:
                float(columns[0])
                float(columns[1])
                float(columns[2])
            except ValueError as exc:
                raise ValueError(
                    f"{path}:{line_number}: first 3 columns must be numeric"
                ) from exc
            points.append((columns[0], columns[1], columns[2]))

    if not points:
        raise ValueError(f"{path}: contains no path points")
    return points


def convert_path_to_pcd(input_path: Path, output_path: Path) -> int:
    points = load_xyz_points(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]

    with output_path.open("w", encoding="ascii") as output_file:
        output_file.write("\n".join(header))
        output_file.write("\n")
        for point in points:
            output_file.write(" ".join(point))
            output_file.write("\n")
    return len(points)


def parse_args():
    input_path = default_input_path()
    parser = argparse.ArgumentParser(
        description="Convert the first XYZ columns of a path file to ASCII PCD."
    )
    parser.add_argument("input", nargs="?", type=Path, default=input_path)
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        help="Default: input file with a .pcd suffix.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_path = args.output or args.input.with_suffix(".pcd")
    point_count = convert_path_to_pcd(args.input, output_path)
    print(f"input: {args.input}")
    print(f"output: {output_path}")
    print(f"points: {point_count}")


if __name__ == "__main__":
    main()
