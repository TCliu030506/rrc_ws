#!/usr/bin/env python3
"""生成沿工具 Z 轴偏置后的 path_map 文件。"""

import argparse
from pathlib import Path

from robot_trajectory_planner.path_map_trajectory_logic import (
    format_path_map_lines,
    offset_path_points_along_tool_z,
    parse_path_map_file,
)


def _default_input_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    package_share = Path(get_package_share_directory('robot_trajectory_planner'))
    return package_share / 'data' / 'path_map.txt'


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Offset every path_map point along its own tool Z axis and write '
            'a new x y z rx ry rz file.'
        ),
    )
    parser.add_argument(
        'offset_distance',
        type=float,
        help='Offset distance in meters. Positive means local +Z.',
    )
    parser.add_argument(
        '-i',
        '--input',
        type=Path,
        default=None,
        help='Input path_map file. Default uses robot_trajectory_planner data/path_map.txt.',
    )
    parser.add_argument(
        '-o',
        '--output',
        type=Path,
        default=None,
        help='Output file. Default writes path_map_offset.txt next to the input file.',
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()
    input_path = args.input.expanduser() if args.input else _default_input_path()
    output_path = (
        args.output.expanduser()
        if args.output
        else input_path.with_name('path_map_offset.txt')
    )

    points = parse_path_map_file(input_path)
    offset_points = offset_path_points_along_tool_z(
        points,
        offset_distance=args.offset_distance,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        '\n'.join(format_path_map_lines(offset_points)) + '\n',
        encoding='utf-8',
    )
    print(
        f'Wrote {len(offset_points)} offset path points to {output_path} '
        f'(offset={args.offset_distance:.6g} m along local +Z).'
    )


if __name__ == '__main__':
    main()
