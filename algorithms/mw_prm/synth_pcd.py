"""Generate a synthetic ``.pcd`` from the analytic obstacles in
``utils/parameters.py`` (or ``utils/parameters_test.py``).

This lets us validate the whole point-cloud pipeline (``main_pcd.py``) on
a *known* scene whose ground-truth obstacles we already have.

What the synthetic cloud contains:
    * a ground patch at z = 0 over (xlim x ylim)
    * uniformly sampled points on every face of every obstacle
    * per-point normals (the analytic face normal, optionally jittered)
    * intensity = obstacle index (handy for ``visualize_pcd.py --color intensity``)

Output is a *binary* PCD with fields:
    x y z intensity normal_x normal_y normal_z curvature
which is exactly the layout your real ``pointcloude/scans_*.pcd`` files use.

Examples
--------
# Generate from the big "obstacles" scene in parameters.py
python synth_pcd.py --params parameters --out pointcloude/synth_params.pcd

# Smaller density + a bit of noise (more realistic looking)
python synth_pcd.py --params parameters --density 80 --noise 0.05 \\
    --out pointcloude/synth_params.pcd

# View it right after
python synth_pcd.py --params parameters --vis

# Plug into the planner
python main_pcd.py --pcd pointcloude/synth_params.pcd \\
    --voxel 0.5 --r-safe 1.0 --rmax 4.0
"""

from __future__ import annotations

import argparse
import importlib
import os
from typing import List, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise ImportError("open3d is required: pip install open3d==0.18.0") from e


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _fan_triangulate(face: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Fan-triangulate a (>=3)-vertex polygon. OK for convex / near-convex faces."""
    tris = []
    v0 = face[0]
    for i in range(1, len(face) - 1):
        tris.append((v0, face[i], face[i + 1]))
    return tris


def _triangle_area(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


def _triangle_normal(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return n / norm


def _sample_triangle(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, n: int, rng: np.random.Generator
) -> np.ndarray:
    """Uniform points inside triangle abc using sqrt-trick barycentric."""
    if n <= 0:
        return np.empty((0, 3))
    r1 = rng.random(n)
    r2 = rng.random(n)
    s1 = np.sqrt(r1)
    u = 1.0 - s1
    v = s1 * (1.0 - r2)
    w = s1 * r2
    return u[:, None] * a + v[:, None] * b + w[:, None] * c


def _orient_outward(n: np.ndarray, face_centroid: np.ndarray, mesh_centroid: np.ndarray) -> np.ndarray:
    """Flip a face normal so it points away from the polyhedron centroid."""
    out_dir = face_centroid - mesh_centroid
    if np.dot(n, out_dir) < 0:
        return -n
    return n


# ---------------------------------------------------------------------------
# Obstacle / ground sampling
# ---------------------------------------------------------------------------

def sample_obstacle(
    obstacle: dict,
    density: float,
    rng: np.random.Generator,
    skip_bottom_face: bool = True,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (points, normals) sampled on every face of an obstacle.

    Faces with all vertices at ``z == 0`` are skipped if
    ``skip_bottom_face`` is True (we already have a ground plane).
    """
    verts = np.asarray(obstacle["vertices"], dtype=np.float64)
    mesh_centroid = verts.mean(axis=0)

    pts_list: List[np.ndarray] = []
    nrm_list: List[np.ndarray] = []

    for face_idx in obstacle["faces"]:
        face = verts[face_idx]
        if skip_bottom_face and np.all(np.abs(face[:, 2]) < 1e-9):
            continue

        tris = _fan_triangulate(face)
        face_centroid = face.mean(axis=0)
        face_n_raw = _triangle_normal(*tris[0])
        face_n = _orient_outward(face_n_raw, face_centroid, mesh_centroid)

        for a, b, c in tris:
            area = _triangle_area(a, b, c)
            n_pts = max(1, int(np.ceil(area * density)))
            tri_pts = _sample_triangle(a, b, c, n_pts, rng)
            pts_list.append(tri_pts)
            nrm_list.append(np.tile(face_n, (n_pts, 1)))

    if not pts_list:
        return np.empty((0, 3)), np.empty((0, 3))
    return np.vstack(pts_list), np.vstack(nrm_list)


def sample_ground(
    xlim: Tuple[float, float],
    ylim: Tuple[float, float],
    density: float,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, np.ndarray]:
    area = (xlim[1] - xlim[0]) * (ylim[1] - ylim[0])
    n = max(1, int(area * density))
    xs = rng.uniform(xlim[0], xlim[1], n)
    ys = rng.uniform(ylim[0], ylim[1], n)
    zs = np.zeros(n)
    pts = np.stack([xs, ys, zs], axis=1)
    nrm = np.tile([0.0, 0.0, 1.0], (n, 1))
    return pts, nrm


# ---------------------------------------------------------------------------
# Carve out points that lie *inside* any other obstacle (avoid floating dust
# inside a solid). Uses a cheap AABB test, plenty good for these convex shapes.
# ---------------------------------------------------------------------------

def filter_inside_other_aabbs(
    pts: np.ndarray, nrm: np.ndarray, obstacles: List[dict], own_index: int
) -> Tuple[np.ndarray, np.ndarray]:
    if len(pts) == 0:
        return pts, nrm
    keep = np.ones(len(pts), dtype=bool)
    for j, obs in enumerate(obstacles):
        if j == own_index:
            continue
        v = np.asarray(obs["vertices"])
        lo = v.min(0) + 1e-3
        hi = v.max(0) - 1e-3
        inside = (
            (pts[:, 0] > lo[0]) & (pts[:, 0] < hi[0]) &
            (pts[:, 1] > lo[1]) & (pts[:, 1] < hi[1]) &
            (pts[:, 2] > lo[2]) & (pts[:, 2] < hi[2])
        )
        keep &= ~inside
    return pts[keep], nrm[keep]


# ---------------------------------------------------------------------------
# PCD writer (binary, with intensity + normals + curvature, matches the
# layout of the real scans in pointcloude/)
# ---------------------------------------------------------------------------

def write_pcd_binary(
    path: str,
    pts: np.ndarray,
    nrm: np.ndarray,
    intensity: np.ndarray,
    curvature: np.ndarray,
) -> None:
    n = len(pts)
    header = (
        "# .PCD v0.7 - synthetic from utils/parameters.py\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        "SIZE 4 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")

    dtype = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("intensity", "<f4"),
        ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
        ("curvature", "<f4"),
    ])
    arr = np.empty(n, dtype=dtype)
    arr["x"] = pts[:, 0]
    arr["y"] = pts[:, 1]
    arr["z"] = pts[:, 2]
    arr["intensity"] = intensity
    arr["nx"] = nrm[:, 0]
    arr["ny"] = nrm[:, 1]
    arr["nz"] = nrm[:, 2]
    arr["curvature"] = curvature

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(header)
        f.write(arr.tobytes())


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument(
        "--params", default="parameters",
        choices=["parameters", "parameters_test"],
        help="which params module under utils/ to use",
    )
    p.add_argument(
        "--layer", default="auto",
        choices=["auto", "obstacles", "real", "connect", "expand"],
        help=(
            "which obstacle list to materialize. "
            "'auto' = 'obstacles' (parameters.py) or 'real_obstacles' "
            "(parameters_test.py)."
        ),
    )
    p.add_argument("--out", default=None,
                   help="output PCD path; auto = pointcloude/synth_<params>.pcd")
    p.add_argument("--density", type=float, default=80.0,
                   help="points per square metre on obstacle faces")
    p.add_argument("--ground-density", type=float, default=20.0,
                   help="points per square metre on the ground plane")
    p.add_argument("--noise", type=float, default=0.03,
                   help="position noise std-dev (m), 0 = perfectly clean")
    p.add_argument("--normal-noise", type=float, default=0.02,
                   help="normal jitter std-dev (rad-ish), 0 = perfect normals")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--vis", action="store_true",
                   help="visualize the result with Open3D after writing")
    return p.parse_args()


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)

    print(f"loading utils.{args.params}.get_parameters() ...")
    mod = importlib.import_module(f"utils.{args.params}")
    params = mod.get_parameters()

    if args.layer == "auto":
        keys = ["obstacles", "real_obstacles"]
    else:
        keys = {
            "obstacles":  ["obstacles"],
            "real":       ["real_obstacles"],
            "connect":    ["connect_obstacles"],
            "expand":     ["expand_obstacles"],
        }[args.layer]

    obstacles = None
    chosen_key = None
    for k in keys:
        if k in params:
            obstacles = params[k]
            chosen_key = k
            break
    if obstacles is None:
        raise RuntimeError(
            f"utils/{args.params}.py has none of {keys}; available keys = "
            f"{[k for k in params if k.endswith('obstacles') or k == 'obstacles']}"
        )

    out_path = args.out or f"pointcloude/synth_{args.params}_{chosen_key}.pcd"

    xlim = params["xlim"]
    ylim = params["ylim"]
    print(f"  layer = '{chosen_key}'  ({len(obstacles)} obstacles)")
    print(f"  xlim  = {xlim}   ylim = {ylim}")
    print(f"  start = {params.get('start')}   end = {params.get('end')}")

    all_pts: List[np.ndarray] = []
    all_nrm: List[np.ndarray] = []
    all_intensity: List[np.ndarray] = []

    print(f"sampling ground ({args.ground_density} pts/m^2) ...")
    g_pts, g_nrm = sample_ground(xlim, ylim, args.ground_density, rng)
    all_pts.append(g_pts)
    all_nrm.append(g_nrm)
    all_intensity.append(np.full(len(g_pts), 10.0))
    print(f"  ground: {len(g_pts)} points")

    for k, obs in enumerate(obstacles):
        pts_k, nrm_k = sample_obstacle(obs, args.density, rng, skip_bottom_face=True)
        pts_k, nrm_k = filter_inside_other_aabbs(pts_k, nrm_k, obstacles, k)
        all_pts.append(pts_k)
        all_nrm.append(nrm_k)
        all_intensity.append(np.full(len(pts_k), 50.0 + 30.0 * k))
        print(f"  obstacle {k}: {len(pts_k)} points")

    pts = np.vstack(all_pts).astype(np.float64)
    nrm = np.vstack(all_nrm).astype(np.float64)
    intensity = np.concatenate(all_intensity).astype(np.float64)
    curvature = np.zeros(len(pts), dtype=np.float64)

    if args.noise > 0:
        pts = pts + rng.normal(scale=args.noise, size=pts.shape)
    if args.normal_noise > 0:
        nrm = nrm + rng.normal(scale=args.normal_noise, size=nrm.shape)
        nrm = nrm / (np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-9)

    print(f"\ntotal points: {len(pts):,}")
    print(f"writing to   : {out_path}")
    write_pcd_binary(out_path, pts, nrm, intensity, curvature)
    print("done.")

    if args.vis:
        print("opening Open3D viewer ...")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.normals = o3d.utility.Vector3dVector(nrm)
        z = pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
        colors = np.stack([
            0.20 + 0.80 * z_norm,
            0.60 - 0.40 * z_norm,
            0.30 + 0.40 * (1 - z_norm),
        ], axis=1)
        pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 1))

        start = np.asarray(params["start"])
        end = np.asarray(params["end"])
        s = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=20)
        s.translate(start); s.paint_uniform_color([0.0, 1.0, 0.0])
        s.compute_vertex_normals()
        e = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=20)
        e.translate(end); e.paint_uniform_color([1.0, 0.0, 0.0])
        e.compute_vertex_normals()

        bbox_min = np.array([xlim[0], ylim[0], 0.0])
        bbox_max = np.array([xlim[1], ylim[1], params["zlim"][1]])
        box = o3d.geometry.AxisAlignedBoundingBox(bbox_min, bbox_max)
        box.color = (0.0, 0.0, 0.0)

        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=max(1.0, 0.05 * float(np.max(bbox_max - bbox_min))),
            origin=bbox_min.tolist(),
        )

        o3d.visualization.draw_geometries(
            [pcd, s, e, box, coord],
            window_name=f"Synthetic PCD ({os.path.basename(out_path)})",
            width=1280, height=900,
        )


if __name__ == "__main__":
    main()
