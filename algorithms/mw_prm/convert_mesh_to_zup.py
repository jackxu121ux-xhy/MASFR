#!/usr/bin/env python3
"""Bake ``up_axis`` rotation into mesh / point PLY assets so ``main_pcd.py`` can use ``--up-axis z``.

ICL-NUIM-style rooms (``livingroom.ply``, ``office.ply``) are commonly y-up; this script
writes z-up copies under ``pointcloud/`` next to unchanged LiDAR-style PCDs such as
``pointcloude/synth_params.pcd``.

Uses the same 3×3 rotations as ``utils.env_pointcloud.up_axis_rotation_matrix``.

Examples
--------
    # Default: convert bundled ICL-NUIM meshes to ./pointcloud/*_zup.ply
    python convert_mesh_to_zup.py --batch-icl-nuim

    python convert_mesh_to_zup.py --input datasets/livingroom.ply \\
        --output pointcloud/livingroom_zup.ply --up-axis y
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "open3d is required: pip install open3d==0.18.0"
    ) from e

# Repo root when this script sits at repo root (MMPRM/convert_mesh_to_zup.py)
REPO_ROOT = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.env_pointcloud import up_axis_rotation_matrix  # noqa: E402


def _ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def convert_file(path_in: str, path_out: str, up_axis: str) -> None:
    """Load PLY as point cloud or triangle mesh, apply rotation, save as PLY."""
    R = up_axis_rotation_matrix(up_axis)
    _ensure_parent_dir(path_out)

    pcd = o3d.io.read_point_cloud(path_in)
    if len(pcd.points) > 0:
        if not np.allclose(R, np.eye(3)):
            pcd.rotate(R, center=(0.0, 0.0, 0.0))
        ok = o3d.io.write_point_cloud(path_out, pcd)
        if not ok:
            raise RuntimeError(f"failed to write point cloud: {path_out}")
        print(f"[ok] point cloud {len(pcd.points):>8} pts → {path_out}")
        return

    mesh = o3d.io.read_triangle_mesh(path_in)
    if len(mesh.vertices) == 0:
        raise ValueError(f"empty or unsupported geometry: {path_in}")
    if not np.allclose(R, np.eye(3)):
        mesh.rotate(R, center=(0.0, 0.0, 0.0))
    mesh.compute_vertex_normals()
    ok = o3d.io.write_triangle_mesh(path_out, mesh)
    if not ok:
        raise RuntimeError(f"failed to write triangle mesh: {path_out}")
    nv = len(mesh.vertices)
    nt = len(mesh.triangles)
    print(f"[ok] triangle mesh {nv:>8} v  {nt:>8} tri → {path_out}")


def parse_args():
    choices = ["z", "+z", "y", "-y", "x", "-z"]
    p = argparse.ArgumentParser(description="Convert PLY assets to z-up on disk.")
    p.add_argument(
        "--input",
        "-i",
        action="append",
        default=[],
        metavar="PATH",
        help="input .ply path (repeat for multiple)",
    )
    p.add_argument(
        "--output",
        "-o",
        action="append",
        default=[],
        metavar="PATH",
        help="output .ply path, aligned with each --input (same order & count)",
    )
    p.add_argument(
        "--output-dir",
        "-d",
        default="pointcloud",
        metavar="DIR",
        help="default directory for outputs when using --batch-icl-nuim (mkdir if needed)",
    )
    p.add_argument(
        "--up-axis",
        default="y",
        choices=choices,
        help="input file's upright axis before conversion (livingroom/office → y)",
    )
    p.add_argument(
        "--batch-icl-nuim",
        action="store_true",
        help=f"convert {REPO_ROOT / 'datasets' / 'livingroom.ply'} and office.ply to DIR/*_zup.ply",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    pairs: list[tuple[str, str]] = []

    if args.batch_icl_nuim:
        out_dir = Path(args.output_dir)
        for name in ("livingroom", "office"):
            pin = REPO_ROOT / "datasets" / f"{name}.ply"
            pout = out_dir / f"{name}_zup.ply"
            pairs.append((str(pin), str(pout)))

    if args.input:
        if len(args.input) != len(args.output):
            raise SystemExit(
                "--input and --output counts must match when using explicit pairs."
            )
        pairs.extend(zip(args.input, args.output))

    if not pairs:
        raise SystemExit("use --batch-icl-nuim and/or repeating --input/--output.")

    for pin, pout in pairs:
        if not os.path.isfile(pin):
            print(f"[skip] missing input: {pin}")
            continue
        convert_file(pin, pout, args.up_axis)


if __name__ == "__main__":
    main()
