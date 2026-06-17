#!/usr/bin/env python3
import argparse
import re
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default="urdf/inspire_hand/FTP_right_hand.urdf")
    parser.add_argument("--out-dir", default="generated")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    urdf_path = (root / args.urdf).resolve()
    out_dir = (root / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    text = urdf_path.read_text(encoding="utf-8")

    def repl(match):
        name = match.group(1)
        mesh_path = (urdf_path.parent / name).resolve()
        return f'filename="file://{mesh_path}"'

    text = re.sub(r'filename="([^":]+\.STL)"', repl, text)
    text = text.replace('<color rgba="1 1 1 1"/>', '<color rgba="0.48 0.54 0.62 1"/>')
    text = text.replace(
        '<color rgba="0.411764705882353 0.411764705882353 0.411764705882353 1"/>',
        '<color rgba="0.08 0.09 0.10 0.18"/>',
    )

    rviz_urdf = out_dir / f"{urdf_path.stem}_rviz.urdf"
    rviz_urdf.write_text(text, encoding="utf-8")

    param_file = out_dir / f"{urdf_path.stem}_robot_description.yaml"
    indented = "\n".join("      " + line for line in text.splitlines())
    param_file.write_text(
        "rviz2:\n"
        "  ros__parameters:\n"
        "    robot_description: |\n"
        f"{indented}\n",
        encoding="utf-8",
    )

    print(rviz_urdf)
    print(param_file)


if __name__ == "__main__":
    main()
