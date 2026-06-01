"""End-to-end MMPRM demo on a real point-cloud scene with Open3D.

Pipeline:
    1. load + denoise + voxel-downsample the PCD; optional ceiling z-crop
       (--vis-strip-ceiling / --vis-z-max) inside build_environment
    2. RANSAC ground / non-ground segmentation
    3. sample PRM nodes (**crawl** = floor walk + lifted walkable surfaces,
       both as crawl / ``wg``-capable samples; **aerial** = flight nodes;
       ``--unified-walk`` fuses floor + surface candidates then splits by height)
    4. build PRM edges with KD-tree based collision checker
    5. multi-modal A*  (utils.path_planning.astar, unchanged)
    6. visualize everything in an Open3D window

Run:
    python main_pcd.py --pcd pointcloude/scans_e.pcd
    python main_pcd.py --yml yml/pcd/livingroom_recent.yml
    # ICL-NUIM y-up meshes baked to z-up (see ``convert_mesh_to_zup.py``):
    python convert_mesh_to_zup.py --batch-icl-nuim
    python main_pcd.py --pcd pointcloud/livingroom_zup.ply --vis-strip-ceiling
    python main_pcd.py --pcd datasets/livingroom.ply --up-axis y --vis-strip-ceiling
    python main_pcd.py --pcd datasets/livingroom.ply --up-axis y --vis-strip-ceiling --unified-walk
    python main_pcd.py --pcd datasets/office.ply --up-axis y --vis-strip-ceiling --vis-ceiling-gap 0.09
    # Viewer raw-cloud coloring (same modes as visualize_pcd): --vis-color rgb --vis-brightness 1.2
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from utils.env_pointcloud import (
    PointCloudCollisionChecker,
    PointCloudEnvironment,
    build_environment,
    sample_aerial_nodes,
    sample_ground_nodes,
    up_axis_rotation_matrix,
)
from utils.graph import (
    connect_nearby_nodes,
    generate_nodes_pointcloud,
    heuristic,
)
from utils.path_planning import astar, edge_is_pure_walk, path_segment_kind, smooth_path

try:
    import open3d as o3d
except ImportError as e:
    raise ImportError("This demo needs open3d. `pip install open3d==0.18.0`") from e

from visualize_pcd import colorize as _vis_colorize
from visualize_pcd import read_pcd_full as _vis_read_pcd_full


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser(*, with_yml: bool = True) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MMPRM demo on a PCD scene")
    if with_yml:
        p.add_argument(
            "--yml",
            "--yaml",
            dest="yml",
            metavar="PATH",
            default=None,
            help="YAML file whose keys are argparse dest names (underscores); "
                 "CLI flags after this still apply and override file values.",
        )
    p.add_argument("--pcd", default="pointcloude/scans_e.pcd")
    p.add_argument("--voxel", type=float, default=0.2,
                   help="voxel down-sample size (m)")
    p.add_argument("--r-safe", type=float, default=0.4,
                   help="collision safety radius (m)")
    p.add_argument("--rmax", type=float, default=4.0,
                   help="PRM connection radius (m)")
    p.add_argument("--num-ground", type=int, default=600,
                   help="# of floor crawl PRM nodes (z≈0, ground cloud); with "
                        "--unified-walk, merged with surface pool then split by height")
    p.add_argument("--num-surface", type=int, default=400,
                   help="# of elevated crawl PRM nodes (lifted walkable surfaces)")
    p.add_argument("--num-aerial", type=int, default=400,
                   help="# of aerial PRM nodes")
    p.add_argument(
        "--unified-walk",
        action="store_true",
        help="fuse floor + elevated walk candidates into one voxel pool of size "
             "num_ground+num_surface, then split by z into floor vs elevated blocks "
             "for graph is_surface / edge tagging.",
    )
    p.add_argument(
        "--walkable-normal-radii",
        type=float,
        nargs="+",
        default=None,
        metavar="R",
        help="override multi-scale normal radii (m) for walkable-surface "
             "detection; default scales with --voxel. Smaller R help wavy "
             "tabletops / sofas.",
    )
    p.add_argument(
        "--z-min-walkable-surface",
        type=float,
        default=0.04,
        help="non-ground points with z below this (after align) are not "
             "walkable-surface candidates (m).",
    )
    # --- 高程图方法参数 (Fankhauser & Hutter 2018) ---
    p.add_argument(
        "--elev-max-roughness",
        type=float,
        default=0.15,
        metavar="M",
        help="高程图: 网格单元内 z 标准差上限 (米)。"
             "超过此值认为表面太粗糙，不可着陆。默认 0.15。",
    )
    p.add_argument(
        "--elev-max-slope",
        type=float,
        default=0.85,
        metavar="S",
        help="高程图: 相邻网格间最大坡度 (dz/dx)。"
             "0.85 ≈ tan(40°)。默认 0.85。",
    )
    p.add_argument(
        "--elev-min-area",
        type=float,
        default=0.25,
        metavar="A",
        help="高程图: 连通区域最小面积 (m²)。"
             "小于此值的区域被视为窗台/栏杆。默认 0.25。",
    )
    p.add_argument("--max-surface-z", type=float, default=18.0)
    p.add_argument("--max-slope-deg", type=float, default=50.0)
    p.add_argument("--ground-thresh", type=float, default=0.15,
                   help="RANSAC distance threshold for ground plane (m).")
    p.add_argument("--ground-clearance", type=float, default=0.3,
                   help="After ground z=0 alignment, drop non-ground obstacle "
                        "points with |z|<=this from the *3D* obstacle cloud only "
                        "(reduces wall-foot clutter / false collisions). "
                        "The 2.5D roof heightmap still uses full non-ground. "
                        "Typical 0.2–0.3 for noisy scans; 0.05–0.1 for clean "
                        "synthetic meshes. 0 disables.")
    p.add_argument("--surface-lift", type=float, default=None,
                   help="optional lift for walkable-surface samples along the "
                        "normal (m). Default keeps samples directly on the "
                        "walkable surface; those surface points are carved "
                        "out of the 3D inflated obstacle cloud.")
    p.add_argument("--walkable-edge-margin", type=float, default=None,
                   help="XY margin (m) kept inflated around each walkable "
                        "surface boundary. Default uses r_safe, so landing "
                        "samples are taken only from the interior of a surface.")
    p.add_argument("--obstacle-radius-filter-radius", type=float, default=0.0,
                   help="optional radius-outlier filter radius (m) applied to "
                        "the final 3D obstacle cloud; 0 disables. Useful for "
                        "removing isolated real-scan noise before inflation.")
    p.add_argument("--obstacle-radius-filter-min-neighbors", type=int, default=0,
                   help="minimum neighbours required within "
                        "--obstacle-radius-filter-radius; 0 disables.")
    p.add_argument("--surface-regression", action="store_true",
                   help="confirm walkable candidates with local PCA plane "
                        "regression before carving them from obstacles.")
    p.add_argument("--surface-regression-radius", type=float, default=0.5,
                   help="neighbourhood radius for local surface regression.")
    p.add_argument("--surface-regression-min-neighbors", type=int, default=12,
                   help="minimum neighbours required for local surface regression.")
    p.add_argument("--surface-regression-max-rmse", type=float, default=0.08,
                   help="maximum local plane residual RMS accepted as planar.")
    p.add_argument("--surface-regression-max-projection", type=float, default=0.15,
                   help="maximum distance from a candidate point to its fitted "
                        "local plane.")
    p.add_argument("--scale", type=float, default=1.0,
                   help="multiply the input cloud's xyz by this factor "
                        "*before* segmentation. Use e.g. 10 to blow up a "
                        "3m indoor scan into a 30m-scale scene so the "
                        "default planner / r_safe / rmax values fit. "
                        "voxel_size is interpreted in the scaled world.")
    p.add_argument("--z-origin", choices=["ground", "min"], default="ground",
                   help="'ground' keeps the detected ground plane at z=0. "
                        "'min' shifts the final environment so the lowest "
                        "point in the point cloud is z=0.")
    p.add_argument("--up-axis", default="z", choices=["z", "y", "x", "-y", "-z"],
                   help="which axis is up in the input cloud. Most LiDAR / "
                        "ROS scans are 'z'. Many CG mesh datasets (incl. "
                        "ICL-NUIM 'livingroom.ply' / 'office.ply') are 'y'. "
                        "Pass this so RANSAC ground detection picks the "
                        "real floor instead of a wall.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--start", type=float, nargs=3, default=None,
                   help="x y z; if omitted, pick from ground samples")
    p.add_argument("--end", type=float, nargs=3, default=None,
                   help="x y z; if omitted, pick from ground samples")

    # ------ multi-modal A* cost weights ------
    p.add_argument("--wg", type=float, default=1.0,
                   help="ground-walking cost per metre (baseline = 1)")
    p.add_argument("--wf", type=float, default=5.0,
                   help="aerial / climbing cost per metre. ratio wf/wg "
                        "controls how strongly the planner prefers walking. "
                        "default 5 -> 1m flying == 5m walking")
    p.add_argument("--e-factor", type=float, default=0.1,
                   help="mode-switch penalty as a fraction of wg "
                        "(e = wg * e_factor). higher => fewer take-off/landing")
    p.add_argument("--smoothing", type=float, default=5.0,
                   help="B-spline smoothing factor passed to smooth_path; "
                        "larger => smoother but less faithful to PRM polyline")
    p.add_argument("--spline-degree", type=int, default=3,
                   help="B-spline degree (2..5). default 3.")
    p.add_argument("--lift-epsilon", type=float, default=1.0,
                   help="vertical lift (m) of the virtual control points "
                        "near take-off / landing on wf segments. larger => "
                        "more vertical take-off & landing; 0 => no lift.")

    # ------ visualization flags ------
    p.add_argument("--no-vis", action="store_true",
                   help="run the planner but skip the Open3D window")
    p.add_argument("--show-prm", action="store_true",
                   help="overlay PRM ground edges (noisy but informative)")
    p.add_argument("--no-samples", action="store_true",
                   help="hide ground/surface/aerial PRM sample points")
    p.add_argument("--bg", default="dark", choices=["dark", "light"],
                   help="window background color")
    p.add_argument("--dim-raw", type=float, default=0.7,
                   help="extra dimming multiplier on scene point colours after "
                        "--vis-color (visualize_pcd has no analogue; keep 1.0 to "
                        "match standalone visualize_pcd brightness for the same "
                        "--vis-brightness). Lower preserves PRM/path contrast.")
    p.add_argument("--keep-rgb", action="store_true",
                   help="if the input PCD has an rgb field (e.g. "
                        "cleaned_scans_*.pcd), keep its original colors "
                        "instead of recoloring by height. dim-raw still "
                        "applies as a brightness scale.")
    p.add_argument("--path-radius", type=float, default=None,
                   help="path tube radius (m); auto = 0.5%% of scene extent")
    p.add_argument("--marker-radius", type=float, default=None,
                   help="start/goal sphere radius (m); auto = 1.5%% of extent")
    p.add_argument("--point-size", type=float, default=2.0,
                   help="point size for the raw cloud (pixels)")
    p.add_argument(
        "--vis-color",
        default="height",
        choices=["height", "intensity", "curvature", "normal", "rgb", "uniform"],
        help="raw-scene point coloring (turbo map / file fields), like "
             "visualize_pcd --color. With --keep-rgb + height mode, file RGB "
             "is kept (legacy).",
    )
    p.add_argument(
        "--vis-brightness",
        type=float,
        default=1.0,
        help="multiply raw-scene colors after dim-raw (>1 brightens); "
             "same idea as visualize_pcd --brightness.",
    )
    p.add_argument(
        "--vis-cloud-voxel",
        type=float,
        default=0.0,
        help="extra voxel downsample on the *display* raw cloud only; "
             "0 = use env.raw_pcd resolution.",
    )
    p.add_argument(
        "--vis-cloud-denoise",
        action="store_true",
        help="statistical outlier removal on the display raw cloud only.",
    )
    p.add_argument(
        "--vis-show-normals",
        action="store_true",
        help="draw normals as line segments on the scene cloud "
             "(use --vis-estimate-normals if needed).",
    )
    p.add_argument(
        "--vis-estimate-normals",
        action="store_true",
        help="estimate normals on the display scene cloud.",
    )
    p.add_argument(
        "--vis-lighting",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="enable Open3D lighting in the viewer. Off by default to match "
             "visualize_pcd.py (lighting often makes coloured points look darker). "
             "Meshes/path tubes render either way.",
    )
    p.add_argument(
        "--vis-recenter",
        action="store_true",
        help="translate all geometries so joint AABB centre is at origin.",
    )
    p.add_argument(
        "--vis-zoom",
        type=float,
        default=0.7,
        help="initial camera zoom (smaller = farther).",
    )
    p.add_argument(
        "--vis-show-bbox",
        action="store_true",
        help="draw axis-aligned bbox around the display scene cloud.",
    )
    p.add_argument(
        "--vis-inflated-obstacles",
        action="store_true",
        help="overlay an explicit red point-cloud approximation of the "
             "r_safe-inflated obstacle cloud.",
    )
    p.add_argument(
        "--vis-inflated-max-centers",
        type=int,
        default=6000,
        help="max obstacle points used as inflation sphere centres for "
             "--vis-inflated-obstacles.",
    )
    p.add_argument(
        "--vis-inflated-mode",
        choices=["sphere", "voxel"],
        default="sphere",
        help="'sphere' samples shells around obstacle points; 'voxel' dilates "
             "an occupancy grid and looks more continuous on walls.",
    )
    p.add_argument(
        "--vis-inflated-voxel",
        type=float,
        default=0.0,
        help="voxel size for --vis-inflated-mode voxel; 0 = auto from r_safe.",
    )
    p.add_argument(
        "--vis-inflated-sphere-samples",
        type=int,
        default=32,
        help="directions sampled on each inflation shell per obstacle centre.",
    )
    p.add_argument(
        "--vis-inflated-shells",
        type=int,
        default=3,
        help="number of concentric shells used to draw the inflated volume.",
    )

    # ------ environment ceiling crop (same cloud for vis, collision, PRM, A*) ------
    p.add_argument(
        "--vis-strip-ceiling",
        action="store_true",
        help="after ground alignment, drop points with z above a cutoff on the "
             "full scene cloud (raw + ground + non-ground). Applies inside "
             "build_environment so collision, PRM, and A* match the viewer. "
             "Tune with --vis-ceiling-gap / --vis-z-max.",
    )
    p.add_argument(
        "--vis-ceiling-gap",
        type=float,
        default=0.07,
        help="with --vis-strip-ceiling: z_cut = z_max - gap*(z_max-z_min) in "
             "aligned coords. Typical mesh rooms: 0.05-0.12.",
    )
    p.add_argument(
        "--vis-z-max",
        type=float,
        default=None,
        help="hard z cap on the environment cloud (aligned coords). Use alone or "
             "with --vis-strip-ceiling (tighter cap wins).",
    )
    p.add_argument(
        "--vis-ceiling-quantile",
        type=float,
        default=None,
        help="optional with --vis-strip-ceiling: z_cut = min(..., "
             "quantile(z, this)). E.g. 0.992 trims a dense top slab.",
    )
    return p


def _norm_yaml_cfg_keys(raw: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for k, v in raw.items():
        if not isinstance(k, str):
            raise ValueError(f"YAML keys must be strings, got {k!r}")
        if k.startswith("__"):
            continue
        nk = k.replace("-", "_")
        if nk in out and out[nk] != v:
            raise ValueError(
                f"duplicate/conflicting YAML key after normalize: {k!r}"
            )
        out[nk] = v
    return out


def yaml_cfg_to_argv(cfg: Dict[str, Any], parser: argparse.ArgumentParser) -> List[str]:
    """Turn a YAML mapping into fake argv entries for ``parser``."""
    normalized = _norm_yaml_cfg_keys(cfg)
    argv: List[str] = []
    for action in parser._actions:
        if isinstance(action, argparse._HelpAction):
            continue
        dest = getattr(action, "dest", None)
        if dest in (None, "help", "yml"):
            continue
        long_opts = [s for s in action.option_strings if s.startswith("--")]
        if not long_opts:
            continue
        flag = max(long_opts, key=len)

        if dest not in normalized:
            continue
        val = normalized[dest]
        if val is None:
            continue

        if isinstance(action, argparse.BooleanOptionalAction):
            opts = [
                x
                for x in action.option_strings
                if x.startswith("--") and x != "--help"
            ]
            if len(opts) >= 2:
                if val is True:
                    argv.append(opts[0])
                elif val is False:
                    argv.append(opts[1])
                else:
                    raise ValueError(
                        f"'{dest}': expected true/false, got {val!r}"
                    )
            continue

        if getattr(action, "nargs", None) == 0 and action.const is True:
            if val is True:
                argv.append(flag)
            elif val is not False:
                raise ValueError(
                    f"'{dest}': expected true/false for boolean flag, got {val!r}"
                )
            continue

        if getattr(action, "nargs", None) == 3:
            if not isinstance(val, (list, tuple)) or len(val) != 3:
                raise ValueError(
                    f"'{dest}': expected length-3 list [x,y,z], got {val!r}"
                )
            argv.append(flag)
            argv.extend(str(float(x)) for x in val)
            continue

        if action.nargs in ("+", "*"):
            if isinstance(val, (list, tuple)):
                seq = [float(x) for x in val]
            else:
                seq = [float(val)]
            if not seq:
                continue
            argv.append(flag)
            argv.extend(str(x) for x in seq)
            continue

        argv.extend([flag, str(val)])
    return argv


def _pop_yml_from_argv(argv: List[str]) -> Tuple[Optional[str], List[str]]:
    """Remove the first ``--yml`` / ``--yaml`` option and return (path, rest)."""
    out: List[str] = []
    yml: Optional[str] = None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("--yml", "--yaml"):
            if i + 1 >= len(argv):
                raise SystemExit(f"{a} requires a path argument")
            yml = argv[i + 1]
            i += 2
            continue
        if a.startswith("--yml=") or a.startswith("--yaml="):
            _, _, val = a.partition("=")
            if not val:
                raise SystemExit(f"{a} requires a non-empty path")
            yml = val
            i += 1
            continue
        out.append(a)
        i += 1
    return yml, out


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    if argv is None:
        argv = sys.argv[1:]
    argv = list(argv)

    cli_yml, remainder = _pop_yml_from_argv(argv)
    if remainder in (["-h"], ["--help"]):
        return build_parser(with_yml=True).parse_args(remainder)

    yargv: List[str] = []
    if cli_yml is not None:
        try:
            import yaml  # type: ignore[import-untyped]
        except ImportError as e:
            raise SystemExit(
                "PyYAML is required when using --yml / --yaml. "
                "Install with: pip install pyyaml"
            ) from e
        yml_path = Path(cli_yml).expanduser()
        if not yml_path.is_file():
            raise SystemExit(f"not a file: {yml_path}")
        raw = yaml.safe_load(yml_path.read_text(encoding="utf-8"))
        if raw is None:
            raw = {}
        if not isinstance(raw, dict):
            raise SystemExit("YAML root must be a mapping (dict)")
        yargv = yaml_cfg_to_argv(dict(raw), build_parser(with_yml=False))

    parser = build_parser(with_yml=False)
    try:
        args = parser.parse_args(yargv + remainder)
    except SystemExit:
        raise
    setattr(args, "yml", cli_yml)
    return args


# ---------------------------------------------------------------------------
# Auto-pick start / goal if user didn't pass them
# ---------------------------------------------------------------------------

def pick_default_start_end(
    ground_pts: np.ndarray,
    surface_pts: np.ndarray,
    aerial_pts: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Start: min (x+y) on floor crawl if any, else elevated crawl, else aerial.

    End: max z on elevated crawl if any, else on floor crawl, else aerial.
    """
    g = np.asarray(ground_pts, dtype=np.float64).reshape(-1, 3)
    s = np.asarray(surface_pts, dtype=np.float64).reshape(-1, 3)
    a = np.asarray(aerial_pts, dtype=np.float64).reshape(-1, 3)
    if len(g) > 0:
        start_pool = g
    elif len(s) > 0:
        start_pool = s
    elif len(a) > 0:
        start_pool = a
    else:
        raise RuntimeError("No crawl or aerial PRM samples; cannot auto-pick start.")
    score = start_pool[:, 0] + start_pool[:, 1]
    start = start_pool[np.argmin(score)]
    if len(s) > 0:
        end = s[np.argmax(s[:, 2])]
    elif len(g) > 0:
        end = g[np.argmax(g[:, 2])]
    elif len(a) > 0:
        end = a[np.argmax(a[:, 2])]
    else:
        end = start_pool[np.argmax(score)]
    return start, end


def snap_to_free(
    p: np.ndarray,
    candidates: np.ndarray,
    checker: PointCloudCollisionChecker,
    k_nearest: int = 256,
) -> np.ndarray:
    """Return ``p`` itself if it's already collision-free, otherwise return
    the closest collision-free candidate among the first ``k_nearest`` by
    Euclidean distance; else fall back to ``p``."""
    p = np.asarray(p, dtype=np.float64)
    if not checker.point_collides(p):
        return p
    if len(candidates) == 0:
        return p
    d = np.linalg.norm(candidates - p, axis=1)
    order = np.argsort(d)
    k = min(int(k_nearest), len(order))
    for i in order[:k]:
        c = candidates[i]
        if not checker.point_collides(c):
            return c
    return p


def _vstack_xyz(*blocks: np.ndarray) -> np.ndarray:
    """Stack non-empty (N,3) arrays; empty inputs skipped."""
    parts = [b for b in blocks if b is not None and len(b) > 0]
    if not parts:
        return np.empty((0, 3))
    return np.vstack(parts)


def merge_unified_crawl_nodes(
    ground_pcd: "o3d.geometry.PointCloud",
    surface_cand: np.ndarray,
    n_walk: int,
    sample_voxel: float,
    rng: np.random.Generator,
    z_floor_max: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Fuse floor + elevated candidates, subsample, then split by z for graph masks.

    Points with ``z <= z_floor_max`` become ``ground_pts`` (floor crawl, not
    ``is_surface``); the rest become ``surface_pts`` (elevated crawl).
    """
    n_walk = max(1, int(n_walk))
    n_g = min(max(n_walk * 4, 800), 4000)
    g_cand = sample_ground_nodes(
        ground_pcd,
        n_samples=n_g,
        voxel_size=sample_voxel,
        rng=rng,
    )
    pool = _vstack_xyz(g_cand, surface_cand)
    if len(pool) == 0:
        return (
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 3), dtype=np.float64),
        )
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pool.astype(np.float64))
    vd = max(float(sample_voxel) * 0.35, 0.06)
    pc = pc.voxel_down_sample(voxel_size=vd)
    pts = np.asarray(pc.points, dtype=np.float64)
    if len(pts) > n_walk:
        idx = rng.choice(len(pts), size=n_walk, replace=False)
        pts = pts[idx]
    z_cut = float(z_floor_max)
    gmask = pts[:, 2] <= z_cut
    return pts[gmask], pts[~gmask]


# ---------------------------------------------------------------------------
# Visualization (Open3D)
# ---------------------------------------------------------------------------

def color_pcd_by_height(
    pcd: "o3d.geometry.PointCloud", dim: float = 1.0
) -> "o3d.geometry.PointCloud":
    """Color a cloud by height with a soft ramp; ``dim`` in [0,1] darkens it."""
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return pcd
    z = pts[:, 2]
    t = (z - z.min()) / (z.max() - z.min() + 1e-9)
    lo = np.array([0.32, 0.45, 0.62])
    hi = np.array([0.78, 0.86, 0.96])
    colors = lo + t[:, None] * (hi - lo)
    colors = np.clip(colors * float(dim), 0.0, 1.0)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def _aligned_xyz_from_original_file_pts(
    xyz: np.ndarray,
    *,
    scale: float,
    up_axis: str,
    T_w2g: np.ndarray,
) -> np.ndarray:
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    s = float(scale)
    if abs(s - 1.0) > 1e-15:
        pc.scale(s, center=(0.0, 0.0, 0.0))
    R_up = up_axis_rotation_matrix(up_axis)
    if not np.allclose(R_up, np.eye(3)):
        pc.rotate(np.asarray(R_up, dtype=np.float64), center=(0.0, 0.0, 0.0))
    pc.transform(np.asarray(T_w2g, dtype=np.float64))
    return np.asarray(pc.points, dtype=np.float64)


def _world_to_aligned_4x4(env: PointCloudEnvironment) -> np.ndarray:
    G = getattr(env, "ground_to_world", None)
    if G is None:
        return np.eye(4, dtype=np.float64)
    return np.linalg.inv(np.asarray(G, dtype=np.float64))


def _per_point_file_fields_nn(
    pts_aligned: np.ndarray,
    src_path: str,
    *,
    scale: float,
    up_axis: str,
    T_w2g: np.ndarray,
    color_mode: str,
) -> Dict[str, np.ndarray]:
    _, fields_dense = _vis_read_pcd_full(src_path)
    xf = np.asarray(fields_dense["x"], dtype=np.float64)
    yf = np.asarray(fields_dense["y"], dtype=np.float64)
    zf = np.asarray(fields_dense["z"], dtype=np.float64)
    if xf.size == 0:
        return {}
    pts_f = np.stack([xf, yf, zf], axis=1)
    tree_xyz = _aligned_xyz_from_original_file_pts(
        pts_f, scale=scale, up_axis=up_axis, T_w2g=T_w2g
    )
    from scipy.spatial import cKDTree

    _, idx = cKDTree(tree_xyz).query(np.asarray(pts_aligned, dtype=np.float64), k=1)
    out: Dict[str, np.ndarray] = {}
    if color_mode == "intensity" and "intensity" in fields_dense:
        out["intensity"] = np.asarray(fields_dense["intensity"], dtype=np.float64)[idx]
    if color_mode == "curvature" and "curvature" in fields_dense:
        out["curvature"] = np.asarray(fields_dense["curvature"], dtype=np.float64)[idx]
    if color_mode == "rgb" and "rgb" in fields_dense:
        out["rgb"] = np.asarray(fields_dense["rgb"], dtype=np.float64)[idx]
    return out


def prepare_display_scene_cloud(
    raw_pcd: o3d.geometry.PointCloud,
    *,
    pcd_path: str,
    environment: Optional[PointCloudEnvironment],
    scale: float,
    up_axis: str,
    vis_color: str,
    vis_brightness: float,
    dim_raw: float,
    keep_rgb: bool,
    vis_cloud_voxel: float,
    vis_cloud_denoise: bool,
    vis_estimate_normals: bool,
    vis_show_normals: bool,
    vis_show_bbox: bool,
    bg_dark: bool,
) -> Tuple[o3d.geometry.PointCloud, List]:
    """Color / downsample the planner's raw cloud for viewing (like ``visualize_pcd``)."""
    pc = o3d.geometry.PointCloud(raw_pcd)
    vv = float(vis_cloud_voxel)
    if vv > 0 and len(pc.points) > 0:
        pc = pc.voxel_down_sample(voxel_size=vv)
        if not pc.has_normals():
            pc.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=max(3 * vv, 0.06), max_nn=30
                )
            )

    if vis_cloud_denoise and len(pc.points) > 0:
        pc, _ = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    need_normals = (
        vis_estimate_normals or vis_show_normals or vis_color == "normal"
    )
    if need_normals and len(pc.points) > 0 and not pc.has_normals():
        rad = max(0.4, 3.0 * (vv if vv > 0 else 0.2))
        pc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=rad, max_nn=30)
        )
        pc.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

    pts = np.asarray(pc.points, dtype=np.float64)
    extra: List = []
    if len(pts) == 0:
        return pc, extra

    sub_fields: Dict[str, np.ndarray] = {}
    if vis_color in ("intensity", "curvature", "rgb") and environment is not None:
        Tw2g = _world_to_aligned_4x4(environment)
        sub_fields = _per_point_file_fields_nn(
            pts,
            pcd_path,
            scale=scale,
            up_axis=up_axis,
            T_w2g=Tw2g,
            color_mode=vis_color,
        )

    nrm = np.asarray(pc.normals, dtype=np.float64) if pc.has_normals() else None

    if keep_rgb and vis_color == "height" and pc.has_colors():
        cols = np.asarray(pc.colors, dtype=np.float64)
        if cols.size and cols.max() > 1e-6:
            cols = np.clip(cols * float(dim_raw) * float(vis_brightness), 0.0, 1.0)
            pc.colors = o3d.utility.Vector3dVector(cols)
        else:
            cols = _vis_colorize(pts, sub_fields, vis_color, nrm)
            cols = np.clip(cols * float(dim_raw) * float(vis_brightness), 0.0, 1.0)
            pc.colors = o3d.utility.Vector3dVector(cols)
    else:
        cols = _vis_colorize(pts, sub_fields, vis_color, nrm)
        cols = np.clip(cols * float(dim_raw) * float(vis_brightness), 0.0, 1.0)
        pc.colors = o3d.utility.Vector3dVector(cols)

    bmin, bmax = pts.min(0), pts.max(0)
    ext = float(np.max(bmax - bmin))

    if vis_show_bbox:
        bb = o3d.geometry.AxisAlignedBoundingBox(bmin, bmax)
        bb.color = (0.85, 0.85, 0.90) if bg_dark else (0.15, 0.15, 0.18)
        extra.append(bb)

    if vis_show_normals and pc.has_normals():
        pts2 = np.asarray(pc.points, dtype=np.float64)
        nrm2 = np.asarray(pc.normals, dtype=np.float64)
        lens = len(pts2)
        rng = np.random.default_rng(0)
        idx = np.arange(lens)
        if lens > 4000:
            idx = rng.choice(lens, size=4000, replace=False)
        ends = pts2[idx] + nrm2[idx] * (0.05 * max(ext, 1e-3))
        line_pts = np.vstack([pts2[idx], ends])
        lines = np.array([[i, i + len(idx)] for i in range(len(idx))], dtype=np.int32)
        ls = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_pts),
            lines=o3d.utility.Vector2iVector(lines),
        )
        ls.colors = o3d.utility.Vector3dVector(
            np.tile([0.95, 0.50, 0.10], (len(lines), 1))
        )
        extra.append(ls)

    return pc, extra


def make_sphere(center, radius=0.6, color=(0.0, 1.0, 0.0), resolution=20):
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=resolution)
    s.translate(np.asarray(center))
    s.paint_uniform_color(color)
    s.compute_vertex_normals()
    return s


def make_pointset(points: np.ndarray, color=(0.0, 0.8, 0.0)):
    if len(points) == 0:
        return o3d.geometry.PointCloud()
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.paint_uniform_color(color)
    return pc


def make_inflated_obstacle_pointcloud(
    obstacle_pcd: o3d.geometry.PointCloud,
    r_safe: float,
    *,
    clear_pcd: Optional[o3d.geometry.PointCloud] = None,
    clear_radius: Optional[float] = None,
    max_centers: int = 6000,
    sphere_samples: int = 32,
    shells: int = 3,
    color=(1.0, 0.05, 0.02),
) -> o3d.geometry.PointCloud:
    """Materialize the implicit ``r_safe`` obstacle inflation as red points."""
    centers = np.asarray(obstacle_pcd.points)
    if len(centers) == 0 or r_safe <= 0.0:
        return o3d.geometry.PointCloud()

    max_centers = max(1, int(max_centers))
    if len(centers) > max_centers:
        rng = np.random.default_rng(12345)
        idx = rng.choice(len(centers), size=max_centers, replace=False)
        centers = centers[idx]

    n_dirs = max(8, int(sphere_samples))
    i = np.arange(n_dirs, dtype=np.float64)
    golden = np.pi * (3.0 - np.sqrt(5.0))
    z = 1.0 - 2.0 * (i + 0.5) / n_dirs
    radius_xy = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    dirs = np.column_stack((
        np.cos(golden * i) * radius_xy,
        np.sin(golden * i) * radius_xy,
        z,
    ))

    shell_radii = np.linspace(
        float(r_safe) / max(1, int(shells)),
        float(r_safe),
        max(1, int(shells)),
    )
    offsets = [np.zeros((1, 3), dtype=np.float64)]
    offsets.extend(dirs * rr for rr in shell_radii)
    offsets_arr = np.vstack(offsets)

    inflated = centers[:, None, :] + offsets_arr[None, :, :]
    inflated = inflated.reshape(-1, 3)

    clear_tree = None
    clear_radius_value = None
    if clear_pcd is not None and len(clear_pcd.points) > 0 and len(inflated) > 0:
        from scipy.spatial import cKDTree

        clear_pts = np.asarray(clear_pcd.points)
        clear_tree = cKDTree(clear_pts)
        clear_radius_value = (
            float(clear_radius)
            if clear_radius is not None
            else max(float(r_safe) * 0.25, 1e-3)
        )
        near_clear = clear_tree.query_ball_point(
            inflated, r=max(clear_radius_value, 1e-3), return_length=True
        ) > 0
        inflated = inflated[~near_clear]

    pc = make_pointset(inflated, color=color)
    # Merge overlapping samples so dense scans stay responsive in Open3D.
    voxel = max(float(r_safe) / 3.0, 1e-3)
    pc = pc.voxel_down_sample(voxel_size=voxel)
    if clear_tree is not None and len(pc.points) > 0:
        pts = np.asarray(pc.points)
        near_clear = clear_tree.query_ball_point(
            pts, r=max(float(clear_radius_value), 1e-3), return_length=True
        ) > 0
        pc = pc.select_by_index(np.where(~near_clear)[0].tolist())
    pc.paint_uniform_color(color)
    return pc


def make_voxel_inflated_obstacle_pointcloud(
    obstacle_pcd: o3d.geometry.PointCloud,
    r_safe: float,
    *,
    clear_pcd: Optional[o3d.geometry.PointCloud] = None,
    clear_radius: Optional[float] = None,
    voxel_size: float = 0.0,
    max_centers: int = 6000,
    color=(1.0, 0.05, 0.02),
) -> o3d.geometry.PointCloud:
    """Materialize inflation as dilated voxel occupancy centers."""
    pts = np.asarray(obstacle_pcd.points)
    if len(pts) == 0 or r_safe <= 0.0:
        return o3d.geometry.PointCloud()

    if len(pts) > max_centers:
        rng = np.random.default_rng(12345)
        pts = pts[rng.choice(len(pts), size=max(1, int(max_centers)), replace=False)]

    v = float(voxel_size) if voxel_size and voxel_size > 0.0 else max(float(r_safe) * 0.35, 0.03)
    v = max(v, 1e-3)
    origin = pts.min(axis=0) - float(r_safe) - v
    keys = np.floor((pts - origin) / v).astype(np.int32)

    n = int(np.ceil(float(r_safe) / v))
    rg = np.arange(-n, n + 1, dtype=np.int32)
    dx, dy, dz = np.meshgrid(rg, rg, rg, indexing="ij")
    offsets = np.column_stack((dx.ravel(), dy.ravel(), dz.ravel()))
    offsets = offsets[np.linalg.norm(offsets.astype(np.float64) * v, axis=1) <= float(r_safe)]

    inflated_keys = (keys[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
    inflated_keys = np.unique(inflated_keys, axis=0)
    inflated = origin + (inflated_keys.astype(np.float64) + 0.5) * v

    if clear_pcd is not None and len(clear_pcd.points) > 0 and len(inflated) > 0:
        from scipy.spatial import cKDTree

        clear_tree = cKDTree(np.asarray(clear_pcd.points))
        radius = (
            float(clear_radius)
            if clear_radius is not None
            else max(v, 1e-3)
        )
        near_clear = clear_tree.query_ball_point(
            inflated, r=max(radius, 1e-3), return_length=True
        ) > 0
        inflated = inflated[~near_clear]

    return make_pointset(inflated, color=color)


def make_lineset(
    pairs: List[Tuple[np.ndarray, np.ndarray]],
    color=(0.6, 0.6, 0.6),
):
    if len(pairs) == 0:
        return o3d.geometry.LineSet()
    pts = []
    lines = []
    for a, b in pairs:
        i = len(pts)
        pts.append(a)
        pts.append(b)
        lines.append([i, i + 1])
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(pts)),
        lines=o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32)),
    )
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
    return ls


# ---------------------------------------------------------------------------
# 3D path rendering: cylinder "tubes" + sphere joints
# ---------------------------------------------------------------------------

def _segment_rotation(direction: np.ndarray) -> np.ndarray:
    """Return R that maps +Z onto the unit ``direction``."""
    z = np.array([0.0, 0.0, 1.0])
    d = direction / (np.linalg.norm(direction) + 1e-12)
    v = np.cross(z, d)
    s = float(np.linalg.norm(v))
    c = float(np.dot(z, d))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s * s))


def make_cylinder_segment(p1, p2, radius, color, resolution=14):
    p1 = np.asarray(p1, dtype=np.float64)
    p2 = np.asarray(p2, dtype=np.float64)
    d = p2 - p1
    length = float(np.linalg.norm(d))
    if length < 1e-9:
        return None
    cyl = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=length, resolution=resolution, split=1
    )
    R = _segment_rotation(d)
    cyl.rotate(R, center=(0.0, 0.0, 0.0))
    cyl.translate(0.5 * (p1 + p2))
    cyl.paint_uniform_color(color)
    cyl.compute_vertex_normals()
    return cyl


def make_path_tubes(
    segments: List[Tuple[np.ndarray, np.ndarray]],
    radius: float,
    color,
    add_joint_spheres: bool = True,
    resolution: int = 14,
):
    """Build one merged TriangleMesh covering all segments.

    Joint spheres at endpoints prevent visible gaps at sharp corners.
    """
    mesh = o3d.geometry.TriangleMesh()
    if not segments:
        return mesh
    seen_endpoints: set = set()
    for a, b in segments:
        cyl = make_cylinder_segment(a, b, radius, color, resolution=resolution)
        if cyl is not None:
            mesh += cyl
        if add_joint_spheres:
            for ep in (a, b):
                key = (round(float(ep[0]), 4), round(float(ep[1]), 4), round(float(ep[2]), 4))
                if key in seen_endpoints:
                    continue
                seen_endpoints.add(key)
                s = o3d.geometry.TriangleMesh.create_sphere(
                    radius=radius * 1.05, resolution=resolution
                )
                s.translate(np.asarray(ep))
                s.paint_uniform_color(color)
                mesh += s
    mesh.compute_vertex_normals()
    return mesh


def make_marker(center, radius, color, kind: str = "sphere"):
    """Build a start / goal marker as a sphere with a vertical 'flag' line."""
    out = o3d.geometry.TriangleMesh()
    s = make_sphere(center, radius=radius, color=color, resolution=24)
    out += s
    pole_h = radius * 6.0
    pole_top = np.asarray(center) + np.array([0.0, 0.0, pole_h])
    pole = make_cylinder_segment(center, pole_top, radius=radius * 0.18, color=color)
    if pole is not None:
        out += pole
    cap = o3d.geometry.TriangleMesh.create_sphere(radius=radius * 0.45, resolution=14)
    cap.translate(pole_top)
    cap.paint_uniform_color(color)
    out += cap
    out.compute_vertex_normals()
    return out


# ---------------------------------------------------------------------------
# Visualization main entry
# ---------------------------------------------------------------------------

def visualize(
    raw_pcd,
    crawl_pts,
    aerial_pts,
    edges,
    weights,
    path_segments_wg,
    path_segments_wf,
    path_waypoints: Optional[List[np.ndarray]],
    start,
    goal,
    bounds,
    *,
    show_prm: bool = False,
    show_samples: bool = True,
    bg: str = "dark",
    dim_raw: float = 0.7,
    point_size: float = 2.0,
    path_radius: Optional[float] = None,
    marker_radius: Optional[float] = None,
    keep_rgb: bool = False,
    environment: Optional[PointCloudEnvironment] = None,
    pcd_path: str = "",
    scale: float = 1.0,
    up_axis: str = "z",
    vis_color: str = "height",
    vis_brightness: float = 1.0,
    vis_cloud_voxel: float = 0.0,
    vis_cloud_denoise: bool = False,
    vis_show_normals: bool = False,
    vis_estimate_normals: bool = False,
    vis_lighting: bool = False,
    vis_recenter: bool = False,
    vis_zoom: float = 0.7,
    vis_show_bbox: bool = False,
    vis_inflated_obstacles: bool = False,
    vis_inflated_max_centers: int = 6000,
    vis_inflated_mode: str = "sphere",
    vis_inflated_voxel: float = 0.0,
    vis_inflated_sphere_samples: int = 32,
    vis_inflated_shells: int = 3,
):
    extent = max(b[1] - b[0] for b in bounds)
    if path_radius is None:
        path_radius = max(0.06, 0.005 * extent)
    if marker_radius is None:
        marker_radius = max(0.18, 0.015 * extent)

    bg_dark = bg == "dark"

    scene_vis, scene_extra = prepare_display_scene_cloud(
        raw_pcd,
        pcd_path=pcd_path,
        environment=environment,
        scale=scale,
        up_axis=up_axis,
        vis_color=vis_color,
        vis_brightness=vis_brightness,
        dim_raw=dim_raw,
        keep_rgb=keep_rgb,
        vis_cloud_voxel=vis_cloud_voxel,
        vis_cloud_denoise=vis_cloud_denoise,
        vis_estimate_normals=vis_estimate_normals,
        vis_show_normals=vis_show_normals,
        vis_show_bbox=vis_show_bbox,
        bg_dark=bg_dark,
    )
    geoms_pcd: List = [scene_vis] + list(scene_extra)

    if vis_inflated_obstacles and environment is not None:
        if vis_inflated_mode == "voxel":
            inflated = make_voxel_inflated_obstacle_pointcloud(
                environment.non_ground_pcd,
                environment.collision_checker.r_safe,
                clear_pcd=environment.landable_pcd,
                clear_radius=environment.collision_checker._walk_clear_radius,
                voxel_size=vis_inflated_voxel,
                max_centers=vis_inflated_max_centers,
            )
        else:
            inflated = make_inflated_obstacle_pointcloud(
                environment.non_ground_pcd,
                environment.collision_checker.r_safe,
                clear_pcd=environment.landable_pcd,
                clear_radius=environment.collision_checker._walk_clear_radius,
                max_centers=vis_inflated_max_centers,
                sphere_samples=vis_inflated_sphere_samples,
                shells=vis_inflated_shells,
            )
        if len(inflated.points) > 0:
            geoms_pcd.append(inflated)
            print(
                "      [vis] inflated obstacles: "
                f"{len(inflated.points)} red points "
                f"(mode={vis_inflated_mode}, "
                f"r_safe={environment.collision_checker.r_safe:.3f})"
            )

    if show_samples:
        crawl_color = (1.00, 0.65, 0.15)
        if len(crawl_pts) > 0:
            geoms_pcd.append(make_pointset(crawl_pts, color=crawl_color))
        geoms_pcd.append(make_pointset(aerial_pts, color=(0.30, 0.70, 1.00)))

    geoms_lines: List = []
    if show_prm:
        max_edges = 6000
        edge_pairs = []
        seen = set()
        for u, neigh in edges.items():
            for v in neigh:
                key = frozenset([u, v])
                if key in seen:
                    continue
                seen.add(key)
                w = weights.get(key, None)
                if w is not None and edge_is_pure_walk(w):
                    edge_pairs.append((np.asarray(u), np.asarray(v)))
                if len(edge_pairs) >= max_edges:
                    break
            if len(edge_pairs) >= max_edges:
                break
        edge_color = (0.55, 0.55, 0.65) if bg_dark else (0.40, 0.40, 0.45)
        geoms_lines.append(make_lineset(edge_pairs, color=edge_color))

    geoms_path: List = []
    wg_color = (0.12, 0.55, 1.00)
    wf_color = (1.00, 0.25, 0.20)
    geoms_path.append(make_path_tubes(path_segments_wg, radius=path_radius, color=wg_color))
    geoms_path.append(make_path_tubes(path_segments_wf, radius=path_radius * 1.05, color=wf_color))

    if path_waypoints:
        wpt_mesh = o3d.geometry.TriangleMesh()
        for p in path_waypoints:
            s = o3d.geometry.TriangleMesh.create_sphere(
                radius=path_radius * 1.6, resolution=14
            )
            s.translate(np.asarray(p))
            s.paint_uniform_color((1.0, 1.0, 1.0) if bg_dark else (0.05, 0.05, 0.05))
            wpt_mesh += s
        wpt_mesh.compute_vertex_normals()
        geoms_path.append(wpt_mesh)

    start_marker = make_marker(start, radius=marker_radius, color=(0.10, 1.0, 0.30))
    goal_marker = make_marker(goal, radius=marker_radius, color=(1.0, 0.20, 0.10))

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(1.0, 0.05 * extent),
        origin=[bounds[0][0], bounds[1][0], bounds[2][0]],
    )

    all_geom = (
        geoms_pcd
        + geoms_lines
        + geoms_path
        + [start_marker, goal_marker, coord]
    )
    if vis_recenter:
        blocks: List[np.ndarray] = []
        for g in all_geom:
            if isinstance(g, o3d.geometry.PointCloud):
                blocks.append(np.asarray(g.points))
            elif isinstance(g, o3d.geometry.LineSet):
                blocks.append(np.asarray(g.points))
            elif isinstance(g, o3d.geometry.TriangleMesh):
                if len(g.vertices) > 0:
                    blocks.append(np.asarray(g.vertices))
        if blocks:
            P = np.concatenate(blocks, axis=0)
            c = (P.min(0) + P.max(0)) * 0.5
            for g in all_geom:
                if hasattr(g, "translate"):
                    g.translate(-c)
            print(f"      [vis] recentered joint AABB centre by {-np.round(c, 3)}")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="MMPRM on point cloud", width=1280, height=900)
    for g in all_geom:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    if opt is None:
        print(
            "    [vis] get_render_option() is None — Open3D has no working GL "
            "context (GLEW / Wayland). Skipping vis.run() to avoid a segfault."
        )
        print(
            "    [vis] Fix: ensure GLFW_PLATFORM=x11 (see run_from_yml.sh) or "
            "run with LIBGL_ALWAYS_SOFTWARE=1; otherwise use --no-vis."
        )
        try:
            vis.destroy_window()
        except Exception:
            pass
        return

    opt.point_size = float(point_size)
    if bg_dark:
        opt.background_color = np.array([0.06, 0.07, 0.10])
        for g in geoms_pcd:
            if isinstance(g, o3d.geometry.AxisAlignedBoundingBox):
                g.color = (0.85, 0.85, 0.90)
                vis.update_geometry(g)
    else:
        opt.background_color = np.array([0.97, 0.97, 0.99])
    opt.light_on = bool(vis_lighting)
    opt.mesh_show_back_face = True

    vc = vis.get_view_control()
    if vc is not None:
        try:
            vc.set_zoom(float(vis_zoom))
        except Exception:
            pass

    print(
        "    [vis] path tube radius = "
        f"{path_radius:.3f} m   marker radius = {marker_radius:.3f} m"
    )
    print("    [vis] press H in the window for keyboard shortcuts; Q to close.")

    try:
        vis.run()
    finally:
        try:
            vis.destroy_window()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Path -> visual segments
# ---------------------------------------------------------------------------

def path_to_segments(
    path,
    weights,
    smoothing_factor: float = 5.0,
    spline_degree: int = 3,
    lift_epsilon: float = 1.0,
):
    """Split A* path into smoothed wg / wf segment lists for visualization.

    Returns: ``(wg_segments, wf_segments, lifted_waypoints)``.

    * ``wg_segments`` / ``wf_segments`` are lists of ``(p1, p2)`` tuples.
    * ``lifted_waypoints`` is a list of 3-vectors -- the *original* A*
      waypoints with their z lifted to match the rendered smoothed curve
      (i.e. the same vertical bias ``smooth_path`` applies internally to
      wf segments). Use these for the white waypoint markers so they
      visually sit on the rendered flight tube.
    """
    if path is None or len(path) < 2:
        return [], [], []

    wg_segs: List[Tuple[np.ndarray, np.ndarray]] = []
    wf_segs: List[Tuple[np.ndarray, np.ndarray]] = []
    lifted_wpts: List[np.ndarray] = []

    state = {"type": None, "pts": []}

    def flush():
        if len(state["pts"]) < 2:
            return
        try:
            sm = smooth_path(
                state["pts"],
                type=state["type"],
                smoothing_factor=smoothing_factor,
                spline_degree=spline_degree,
                lift_epsilon=lift_epsilon,
            )
        except Exception:
            sm = np.asarray(state["pts"])
        target = wg_segs if state["type"] == "wg" else wf_segs
        for i in range(len(sm) - 1):
            target.append((np.asarray(sm[i]), np.asarray(sm[i + 1])))

        # Compute lift-biased original waypoints for marker rendering.
        # smooth_path mirrors the same formula on wf segments:
        #     z[1..-2] += num_control * epsilon ;   num_control = len(seg)//3
        # On wg segments smooth_path keeps original waypoint heights,
        # so we use them as-is.
        seg_pts = np.asarray(state["pts"], dtype=np.float64)
        if state["type"] == "wf" and len(seg_pts) >= 3:
            num_control = int(len(seg_pts) / 3)
            lifted = seg_pts.copy()
            lifted[1:-1, 2] += num_control * float(lift_epsilon)
        else:
            lifted = seg_pts.copy()
        # Avoid duplicating the joint waypoint that connects two adjacent
        # same-direction segments (b == next a)
        if lifted_wpts:
            for q in lifted[1:]:
                lifted_wpts.append(q)
        else:
            for q in lifted:
                lifted_wpts.append(q)

    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        w = weights[frozenset([a, b])]
        kind = path_segment_kind(w)

        if state["type"] is None:
            state["type"] = kind
            state["pts"] = [a]

        if kind != state["type"]:
            flush()
            state["type"] = kind
            state["pts"] = [a]

        state["pts"].append(b)

    flush()
    return wg_segs, wf_segs, lifted_wpts


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args: Optional[argparse.Namespace] = None):
    """Run the full demo. Pass ``args`` from YAML/CI to skip ``sys.argv`` parsing."""
    if args is None:
        args = parse_args()
    rng = np.random.default_rng(args.seed)
    np.random.seed(args.seed)

    print(f"[1/5] loading {args.pcd} ...")
    env = build_environment(
        args.pcd,
        voxel_size=args.voxel,
        r_safe=args.r_safe,
        ground_distance_threshold=args.ground_thresh,
        ground_clearance=args.ground_clearance,
        walkable_max_height=args.max_surface_z,
        walkable_max_slope_deg=args.max_slope_deg,
        surface_lift_distance=args.surface_lift,
        scale=args.scale,
        z_origin=args.z_origin,
        up_axis=args.up_axis,
        strip_ceiling=args.vis_strip_ceiling,
        ceiling_rel_gap=args.vis_ceiling_gap,
        ceiling_z_max=args.vis_z_max,
        ceiling_quantile=args.vis_ceiling_quantile,
        walkable_normal_radii=(
            tuple(args.walkable_normal_radii)
            if args.walkable_normal_radii is not None
            else None
        ),
        walkable_edge_margin=args.walkable_edge_margin,
        obstacle_radius_filter_radius=args.obstacle_radius_filter_radius,
        obstacle_radius_filter_min_neighbors=args.obstacle_radius_filter_min_neighbors,
        surface_regression=args.surface_regression,
        surface_regression_radius=args.surface_regression_radius,
        surface_regression_min_neighbors=args.surface_regression_min_neighbors,
        surface_regression_max_rmse=args.surface_regression_max_rmse,
        surface_regression_max_projection=args.surface_regression_max_projection,
        z_min_walkable_surface=args.z_min_walkable_surface,
        # 高程图方法参数
        elevation_map_max_roughness=args.elev_max_roughness,
        elevation_map_max_slope=args.elev_max_slope,
        elevation_map_min_area=args.elev_min_area,
    )
    if args.scale != 1.0:
        print(f"      scale         : x{args.scale}  (raw cloud blown up before processing)")
    if args.z_origin != "ground":
        print(f"      z origin      : {args.z_origin}")
    print(f"      raw points    : {len(env.raw_pcd.points):>8d}")
    print(f"      ground points : {len(env.ground_pcd.points):>8d}")
    print(f"      non-ground    : {len(env.non_ground_pcd.points):>8d}")
    if args.surface_regression:
        print(f"      walkable raw  : {env.walkable_before_regression_count:>8d}")
        print(f"      regression ok : {env.regression_count:>8d}")
    else:
        print(f"      walkable raw  : {env.walkable_count:>8d}")
    print(f"      landable core : {env.landable_count:>8d}")
    print(f"      surface pts   : {len(env.surface_pcd.points):>8d}")
    print(f"      bounds        : x={env.xlim} y={env.ylim} z={env.zlim}")

    print("[2/5] sampling PRM nodes ...")
    # Sampling voxel must be small enough that we actually get the requested
    # node count out of the available cloud. Scale with the *scene extent*
    # rather than just the preprocessing voxel, so small (3 m) and big
    # (100 m) scenes both work without manual tuning.
    extent = max(b[1] - b[0] for b in env.bounds)
    sample_voxel = max(2 * args.voxel, min(0.4, extent / 30.0))

    g_arr_for_z = np.asarray(env.ground_pcd.points, dtype=np.float64)
    ground_z_ref = (
        float(np.median(g_arr_for_z[:, 2]))
        if len(g_arr_for_z) > 0
        else 0.0
    )

    if len(env.surface_pcd.points) > 0:
        surf_ds = env.surface_pcd.voxel_down_sample(sample_voxel)
        surface_cand = np.asarray(surf_ds.points)
        z_lo = max(0.02, float(args.z_min_walkable_surface) * 0.5)
        surface_cand = surface_cand[
            (surface_cand[:, 2] <= ground_z_ref + args.max_surface_z)
            & (surface_cand[:, 2] > ground_z_ref + z_lo)
        ]
    else:
        surface_cand = np.empty((0, 3))

    z_split = ground_z_ref + max(0.03, float(args.z_min_walkable_surface) * 0.75)

    if args.unified_walk:
        n_walk = int(args.num_ground) + int(args.num_surface)
        ground_pts, surface_pts = merge_unified_crawl_nodes(
            env.ground_pcd,
            surface_cand,
            n_walk,
            sample_voxel,
            rng,
            z_floor_max=z_split,
        )
    else:
        ground_pts = sample_ground_nodes(
            env.ground_pcd,
            n_samples=args.num_ground,
            voxel_size=sample_voxel,
            rng=rng,
        )
        surface_pts = surface_cand
        if len(surface_pts) > args.num_surface:
            idx = rng.choice(len(surface_pts), size=args.num_surface, replace=False)
            surface_pts = surface_pts[idx]

    aerial_pts = sample_aerial_nodes(
        n_samples=args.num_aerial,
        bounds=env.bounds,
        collision_checker=env.collision_checker,
        rng=rng,
    )
    if args.unified_walk:
        print(
            f"      crawl unified: floor / elevated / aerial = "
            f"{len(ground_pts)} / {len(surface_pts)} / {len(aerial_pts)}"
        )
    else:
        print(
            f"      crawl: floor / elevated / aerial = "
            f"{len(ground_pts)} / {len(surface_pts)} / {len(aerial_pts)}"
        )

    s_def, e_def = pick_default_start_end(ground_pts, surface_pts, aerial_pts)
    start = np.array(args.start) if args.start is not None else s_def
    end = np.array(args.end) if args.end is not None else e_def
    start_cands = _vstack_xyz(ground_pts, surface_pts, aerial_pts)
    end_cands = _vstack_xyz(surface_pts, ground_pts, aerial_pts)
    start = snap_to_free(start, start_cands, env.collision_checker)
    end = snap_to_free(end, end_cands, env.collision_checker)
    print(f"      start = {np.round(start, 2)}   goal = {np.round(end, 2)}")

    params = {
        "num_samples": args.num_ground + args.num_surface + args.num_aerial,
        "ground_ratio": args.num_ground
        / max(1, args.num_ground + args.num_surface + args.num_aerial),
        "R_max": args.rmax,
        "wg": args.wg,
        "wf": args.wf,
        "e_factor": args.e_factor,
        "start": tuple(start.tolist()),
        "end": tuple(end.tolist()),
        "xlim": env.xlim,
        "ylim": env.ylim,
        "zlim": env.zlim,
    }
    print(
        f"      cost model    : wg={args.wg}  wf={args.wf}  "
        f"e_factor={args.e_factor}  "
        f"(smooth s={args.smoothing}, k={args.spline_degree}, "
        f"lift_eps={args.lift_epsilon})"
    )

    samples, start_idx, end_idx, is_surface = generate_nodes_pointcloud(
        params, ground_pts, surface_pts, aerial_pts
    )

    if len(surface_pts) > 0:
        from scipy.spatial import cKDTree
        s_tree = cKDTree(surface_pts)
        d_end, _ = s_tree.query(end[None, :], k=1)
        if d_end[0] < 1.0:
            is_surface[end_idx] = True

    print(f"[3/5] connecting PRM edges (R={args.rmax}, N={len(samples)}) ...")
    edges, weights = connect_nearby_nodes(
        samples, params,
        collision_checker=env.collision_checker,
        is_surface=is_surface,
    )
    print(f"      total nodes : {len(samples)}")
    print(f"      total edges : {sum(len(v) for v in edges.values()) // 2}")

    print("[4/5] running multi-modal A* ...")
    path = astar(
        tuple(samples[start_idx]),
        tuple(samples[end_idx]),
        edges, weights, params,
    )
    if path is None:
        print("      ! no path found. try larger --rmax or more samples.")
    else:
        total_len = sum(heuristic(path[i], path[i + 1]) for i in range(len(path) - 1))
        print(f"      path waypoints : {len(path)}")
        print(f"      path length    : {total_len:.2f} m")

    print("[5/5] preparing visualization ...")
    if path:
        wg_segs, wf_segs, lifted_wpts = path_to_segments(
            path,
            weights,
            smoothing_factor=args.smoothing,
            spline_degree=args.spline_degree,
            lift_epsilon=args.lift_epsilon,
        )
        waypoints = lifted_wpts
    else:
        wg_segs, wf_segs = [], []
        waypoints = None

    if args.no_vis:
        print("      --no-vis given, skipping window.")
        return

    crawl_vis = _vstack_xyz(ground_pts, surface_pts)
    visualize(
        env.raw_pcd,
        crawl_vis, aerial_pts,
        edges, weights,
        wg_segs, wf_segs,
        waypoints,
        start, end,
        env.bounds,
        show_prm=args.show_prm,
        show_samples=not args.no_samples,
        bg=args.bg,
        dim_raw=args.dim_raw,
        keep_rgb=args.keep_rgb,
        point_size=args.point_size,
        path_radius=args.path_radius,
        marker_radius=args.marker_radius,
        environment=env,
        pcd_path=args.pcd,
        scale=args.scale,
        up_axis=args.up_axis,
        vis_color=args.vis_color,
        vis_brightness=args.vis_brightness,
        vis_cloud_voxel=args.vis_cloud_voxel,
        vis_cloud_denoise=args.vis_cloud_denoise,
        vis_show_normals=args.vis_show_normals,
        vis_estimate_normals=args.vis_estimate_normals,
        vis_lighting=args.vis_lighting,
        vis_recenter=args.vis_recenter,
        vis_zoom=args.vis_zoom,
        vis_show_bbox=args.vis_show_bbox,
        vis_inflated_obstacles=args.vis_inflated_obstacles,
        vis_inflated_max_centers=args.vis_inflated_max_centers,
        vis_inflated_mode=args.vis_inflated_mode,
        vis_inflated_voxel=args.vis_inflated_voxel,
        vis_inflated_sphere_samples=args.vis_inflated_sphere_samples,
        vis_inflated_shells=args.vis_inflated_shells,
    )


if __name__ == "__main__":
    main()
