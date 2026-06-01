"""Build a simple indoor PCD scene out of the Stanford bunny mesh.

Layout: a 12m x 8m flat floor with several scaled bunnies placed as
'sculpture obstacles'. The result is saved as a .pcd compatible with
the rest of the MMPRM pipeline (main_pcd.py / visualize_pcd.py).

Usage:
    python datasets/build_bunny_scene.py
    python main_pcd.py --pcd pointcloude/bunny_room.pcd \\
        --voxel 0.06 --r-safe 0.15 --rmax 1.2 \\
        --num-ground 800 --num-aerial 800 \\
        --max-surface-z 1.6 --start -5 -3 0 --end 5 3 0 \\
        --bg dark --color-by-height
"""

from __future__ import annotations

import os
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise ImportError("open3d is required") from e


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BUNNY_PLY = os.path.join(HERE, "bunny", "reconstruction", "bun_zipper.ply")
OUT_PCD = os.path.join(ROOT, "pointcloude", "bunny_room.pcd")


def load_bunny_points(n_points: int = 8000) -> np.ndarray:
    """Load the bunny mesh and uniformly sample it into a point cloud."""
    if not os.path.isfile(BUNNY_PLY):
        raise FileNotFoundError(
            f"Could not find {BUNNY_PLY}. Run the download steps first."
        )
    mesh = o3d.io.read_triangle_mesh(BUNNY_PLY)
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_uniformly(number_of_points=n_points)
    pts = np.asarray(pcd.points)
    # Stanford bunny is ~0.16 m tall; recenter on its base centroid
    pts -= pts.mean(0)
    return pts


def place_bunny(pts: np.ndarray, scale: float, xyz: np.ndarray, yaw_deg: float):
    """Scale + rotate (about z) + translate a copy of the bunny."""
    out = pts.copy() * scale
    a = np.radians(yaw_deg)
    R = np.array([[np.cos(a), -np.sin(a), 0.0],
                  [np.sin(a),  np.cos(a), 0.0],
                  [0.0,        0.0,       1.0]])
    out = out @ R.T
    # Lift the cloud so its lowest z is on the floor
    out[:, 2] -= out[:, 2].min()
    out += xyz
    return out


def make_floor(xlim, ylim, density: float = 60.0) -> np.ndarray:
    """A noisy patch of points at z=0 covering xlim x ylim. density = pts/m^2."""
    area = (xlim[1] - xlim[0]) * (ylim[1] - ylim[0])
    n = int(area * density)
    rng = np.random.default_rng(0)
    xs = rng.uniform(xlim[0], xlim[1], n)
    ys = rng.uniform(ylim[0], ylim[1], n)
    zs = rng.normal(0.0, 0.005, n)
    return np.stack([xs, ys, zs], axis=1)


def build_scene() -> o3d.geometry.PointCloud:
    bunny = load_bunny_points(n_points=12000)

    parts = [make_floor(xlim=(-6, 6), ylim=(-4, 4), density=120.0)]

    # Each bunny scaled to roughly 1.0 m tall (default bun_zipper is ~0.16 m)
    # so they look like statues a humanoid robot would have to navigate.
    bunny_height = float(bunny[:, 2].max() - bunny[:, 2].min())
    target_h = 1.0
    scale = target_h / max(bunny_height, 1e-6)

    # Place 5 bunnies at fixed locations in the floor box
    placements = [
        ((-3.5, -1.5, 0.0),   0.0,   1.0),
        (( 3.5,  1.5, 0.0),  90.0,   1.0),
        (( 0.0,  0.0, 0.0), 180.0,   1.4),  # bigger one in the middle
        ((-1.5,  2.0, 0.0), 235.0,   0.8),
        (( 2.0, -2.0, 0.0),  45.0,   0.8),
    ]
    for (xyz, yaw, sf) in placements:
        parts.append(place_bunny(bunny, scale=scale * sf, xyz=np.asarray(xyz), yaw_deg=yaw))

    pts = np.concatenate(parts, axis=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=15)
    )
    pcd.orient_normals_to_align_with_direction([0.0, 0.0, 1.0])

    # Color by z so visualize_pcd.py default rendering looks nice
    z = np.asarray(pcd.points)[:, 2]
    t = (z - z.min()) / max(z.max() - z.min(), 1e-9)
    cols = np.stack([0.4 + 0.6 * t, 0.4 + 0.4 * (1 - t), 0.6 * (1 - t)], axis=1)
    pcd.colors = o3d.utility.Vector3dVector(np.clip(cols, 0.0, 1.0))

    return pcd


def main():
    pcd = build_scene()
    os.makedirs(os.path.dirname(OUT_PCD), exist_ok=True)
    ok = o3d.io.write_point_cloud(OUT_PCD, pcd, write_ascii=False)
    if not ok:
        print(f"failed to write {OUT_PCD}")
        sys.exit(1)
    pts = np.asarray(pcd.points)
    bmin, bmax = pts.min(0), pts.max(0)
    print(f"wrote {OUT_PCD}")
    print(f"  points : {len(pts):,}")
    print(f"  bounds : x=[{bmin[0]:.2f},{bmax[0]:.2f}] "
          f"y=[{bmin[1]:.2f},{bmax[1]:.2f}] z=[{bmin[2]:.2f},{bmax[2]:.2f}]")
    print(f"  size   : {(bmax-bmin)[0]:.1f} x {(bmax-bmin)[1]:.1f} x {(bmax-bmin)[2]:.1f} m")


if __name__ == "__main__":
    main()
