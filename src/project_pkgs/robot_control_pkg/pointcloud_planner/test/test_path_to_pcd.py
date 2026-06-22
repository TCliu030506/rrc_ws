import importlib.util
from pathlib import Path


def load_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "path_to_pcd.py"
    )
    spec = importlib.util.spec_from_file_location("path_to_pcd", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_convert_path_to_pcd_uses_first_three_columns(tmp_path):
    module = load_module()
    input_path = tmp_path / "path.txt"
    output_path = tmp_path / "path.pcd"
    input_path.write_text(
        "0.1 0.2 0.3 1 2 3\n"
        "0.4 0.5 0.6 4 5 6\n",
        encoding="ascii",
    )

    point_count = module.convert_path_to_pcd(input_path, output_path)
    content = output_path.read_text(encoding="ascii")

    assert point_count == 2
    assert "WIDTH 2" in content
    assert "POINTS 2" in content
    assert content.endswith("0.1 0.2 0.3\n0.4 0.5 0.6\n")
