"""Standalone visualizer for the PCDs in ``pointcloude/``.

Examples
--------
# View one file with default (height) coloring
python visualize_pcd.py --pcd pointcloude/scans_e.pcd

# Color by intensity (the PCD has an "intensity" field)
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --color intensity

# Show estimated normals as little hairs
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --color normal --show-normals

# Compare all three scans side-by-side (each shifted along +X)
python visualize_pcd.py --compare pointcloude/scans_e.pcd pointcloude/scans_q.pcd pointcloude/scans_w.pcd

# Drop big outliers and downsample for fast inspection
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --voxel 0.1 --denoise

# Print stats only (no window)
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --no-vis

# Defaults from YAML (CLI overrides YAML)
python visualize_pcd.py --yml yml/visualize/cleaned_scans_e.yml
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise ImportError(
        "open3d is required: pip install open3d==0.18.0"
    ) from e


# ---------------------------------------------------------------------------
# PCD loading (incl. extra fields like intensity that Open3D drops by default)
# ---------------------------------------------------------------------------

def parse_pcd_header(path: str) -> Tuple[dict, int]:
    """Return (header_dict, header_byte_length)."""
    header: dict = {}
    n_bytes = 0
    with open(path, "rb") as f:
        for raw in f:
            n_bytes += len(raw)
            line = raw.decode("ascii", errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key.strip()] = value.strip()
            if key.strip() == "DATA":
                break
    return header, n_bytes


def read_pcd_full(path: str) -> Tuple[np.ndarray, dict]:
    """Read a point cloud and return its fields.
    For .pcd we parse the binary/ascii layout directly (so we keep
    intensity / curvature / rgb fields). For anything else (.ply,
    .xyz, .xyzn, .pts) we fall back to Open3D's loader, which gives us
    xyz + (optional) colors + (optional) normals.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext != ".pcd":
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        fields_dict: dict = {"x": pts[:, 0], "y": pts[:, 1], "z": pts[:, 2]}
        if pcd.has_colors():
            cols = np.asarray(pcd.colors)
            fields_dict["rgb"] = cols  # shape (N, 3) float in [0, 1]
        if pcd.has_normals():
            nrm = np.asarray(pcd.normals)
            fields_dict["normal_x"] = nrm[:, 0]
            fields_dict["normal_y"] = nrm[:, 1]
            fields_dict["normal_z"] = nrm[:, 2]
        return pts, fields_dict

    header, header_bytes = parse_pcd_header(path)
    fields = header["FIELDS"].split()
    sizes = [int(x) for x in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(x) for x in header["COUNT"].split()]
    n_points = int(header["POINTS"])
    data_kind = header["DATA"]

    if any(c != 1 for c in counts):
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        return pts, {"x": pts[:, 0], "y": pts[:, 1], "z": pts[:, 2]}

    np_types = {("F", 4): "<f4", ("F", 8): "<f8",
                ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
                ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8"}
    dtype = np.dtype([
        (name, np_types[(t, s)]) for name, t, s in zip(fields, types, sizes)
    ])

    if data_kind == "binary":
        with open(path, "rb") as f:
            f.seek(header_bytes)
            buf = f.read(dtype.itemsize * n_points)
        arr = np.frombuffer(buf, dtype=dtype, count=n_points)
    elif data_kind == "ascii":
        arr = np.loadtxt(path, skiprows=header_bytes_to_lines(path),
                         dtype=dtype)
    else:
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        return pts, {"x": pts[:, 0], "y": pts[:, 1], "z": pts[:, 2]}

    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    fields_dict = {name: np.asarray(arr[name]) for name in fields}
    return pts, fields_dict


def header_bytes_to_lines(path: str) -> int:
    n = 0
    with open(path, "r", errors="ignore") as f:
        for line in f:
            n += 1
            if line.strip().startswith("DATA"):
                break
    return n


# ---------------------------------------------------------------------------
# Colorization
# ---------------------------------------------------------------------------

def _normalize(v: np.ndarray, percentile: float = 1.0) -> np.ndarray:
    """Normalize to [0, 1] using the [percentile, 100-percentile] range.

    Using percentile clipping (instead of raw min/max) prevents a small
    number of outlier points (e.g. floor noise) from squashing the rest
    of the data into a tiny corner of the colormap.
    """
    v = np.asarray(v, dtype=np.float64)
    lo = float(np.nanpercentile(v, percentile))
    hi = float(np.nanpercentile(v, 100.0 - percentile))
    if hi - lo < 1e-9:
        lo, hi = float(np.nanmin(v)), float(np.nanmax(v))
        if hi - lo < 1e-9:
            return np.zeros_like(v)
    out = (v - lo) / (hi - lo)
    out = np.clip(out, 0.0, 1.0)
    return np.nan_to_num(out, nan=0.0)


def _turbo_lut() -> np.ndarray:
    # Skip the very-dark-purple first stop of the standard turbo lut so
    # that even points at the bottom of the range stay visually bright.
    return np.array([
        [0.27628, 0.42118, 0.89093],
        [0.20860, 0.67867, 0.95369],
        [0.32301, 0.91955, 0.65448],
        [0.74879, 0.97836, 0.28725],
        [0.96583, 0.74185, 0.16492],
        [0.93336, 0.40313, 0.10398],
        [0.61853, 0.10072, 0.02475],
    ])


def cmap_turbo(t: np.ndarray) -> np.ndarray:
    lut = _turbo_lut()
    n = len(lut) - 1
    t = np.clip(t, 0.0, 1.0)
    pos = t * n
    i0 = np.floor(pos).astype(int)
    i1 = np.clip(i0 + 1, 0, n)
    a = (pos - i0)[:, None]
    return lut[i0] * (1 - a) + lut[i1] * a


def colorize(
    pts: np.ndarray,
    fields: dict,
    mode: str,
    normals: Optional[np.ndarray] = None,
) -> np.ndarray:
    if mode == "height":
        return cmap_turbo(_normalize(pts[:, 2]))
    if mode == "intensity":
        if "intensity" in fields:
            v = np.asarray(fields["intensity"], dtype=np.float64)
            return cmap_turbo(_normalize(v))
        return cmap_turbo(_normalize(pts[:, 2]))
    if mode == "curvature":
        if "curvature" in fields:
            v = np.asarray(fields["curvature"], dtype=np.float64)
            return cmap_turbo(_normalize(np.log1p(np.abs(v))))
        return cmap_turbo(_normalize(pts[:, 2]))
    if mode == "normal":
        if normals is None:
            return np.full_like(pts, 0.6)
        n = np.asarray(normals)
        return np.clip(0.5 + 0.5 * n, 0.0, 1.0)
    if mode == "rgb":
        if "rgb" in fields:
            rgb_raw = np.asarray(fields["rgb"], dtype=np.float64)
            # PCD rgb can be packed as a float (uint32 reinterpreted) or
            # already split into 3 floats. Detect both.
            if rgb_raw.ndim == 2 and rgb_raw.shape[1] == 3:
                cols = rgb_raw
            else:
                # interpret each float as packed uint32 0xAARRGGBB or 0x00RRGGBB
                packed = rgb_raw.astype(np.float32).view(np.uint32)
                r = ((packed >> 16) & 0xFF).astype(np.float64) / 255.0
                g = ((packed >> 8) & 0xFF).astype(np.float64) / 255.0
                b = (packed & 0xFF).astype(np.float64) / 255.0
                cols = np.stack([r, g, b], axis=1)
            # If everything decoded to ~black, fall back to height
            if cols.max() < 1e-6:
                return cmap_turbo(_normalize(pts[:, 2]))
            return np.clip(cols, 0.0, 1.0)
        return cmap_turbo(_normalize(pts[:, 2]))
    if mode == "uniform":
        return np.tile(np.array([0.55, 0.55, 0.60]), (len(pts), 1))
    raise ValueError(f"unknown color mode: {mode}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def make_coord_frame(bbox_min: np.ndarray, size: float) -> "o3d.geometry.TriangleMesh":
    return o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=size, origin=bbox_min.tolist()
    )


def make_axis_aligned_box(bbox_min: np.ndarray, bbox_max: np.ndarray):
    box = o3d.geometry.AxisAlignedBoundingBox(bbox_min, bbox_max)
    box.color = (0.0, 0.0, 0.0)
    return box


def print_stats(name: str, pts: np.ndarray, fields: dict):
    bmin, bmax = pts.min(0), pts.max(0)
    print(f"\n[{name}]")
    print(f"  fields  : {list(fields.keys())}")
    print(f"  points  : {len(pts):,}")
    print(f"  bounds  :")
    print(f"    x  [{bmin[0]:9.3f}, {bmax[0]:9.3f}]  (width={bmax[0]-bmin[0]:.3f})")
    print(f"    y  [{bmin[1]:9.3f}, {bmax[1]:9.3f}]  (width={bmax[1]-bmin[1]:.3f})")
    print(f"    z  [{bmin[2]:9.3f}, {bmax[2]:9.3f}]  (width={bmax[2]-bmin[2]:.3f})")
    if "intensity" in fields:
        v = fields["intensity"]
        print(f"  intensity: min={float(v.min()):.3f} mean={float(v.mean()):.3f} max={float(v.max()):.3f}")
    if "curvature" in fields:
        v = fields["curvature"]
        print(f"  curvature: min={float(v.min()):.3f} mean={float(v.mean()):.3f} max={float(v.max()):.3f}")


def build_geometries_for_one(
    path: str,
    color_mode: str,
    voxel: float,
    denoise: bool,
    show_normals: bool,
    estimate_normals: bool,
    offset: np.ndarray,
    show_bbox: bool,
):
    pts, fields = read_pcd_full(path)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    if all(k in fields for k in ("normal_x", "normal_y", "normal_z")):
        n = np.stack([fields["normal_x"], fields["normal_y"], fields["normal_z"]], axis=1)
        pcd.normals = o3d.utility.Vector3dVector(n.astype(np.float64))

    if voxel and voxel > 0:
        pcd_ds = pcd.voxel_down_sample(voxel)
        if pcd.has_normals() and not pcd_ds.has_normals():
            pcd_ds.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=3 * voxel, max_nn=30)
            )
        pcd = pcd_ds

    if denoise:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    if estimate_normals and not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(0.4, 3 * (voxel or 0.2)), max_nn=30
            )
        )
        pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

    pts_after = np.asarray(pcd.points)
    nrm_after = np.asarray(pcd.normals) if pcd.has_normals() else None
    sub_fields = {k: np.full(len(pts_after), np.nan) for k in fields if k not in ("x", "y", "z")}
    if voxel <= 0 and not denoise:
        sub_fields = {k: v for k, v in fields.items() if k not in ("x", "y", "z")}

    colors = colorize(pts_after, sub_fields, color_mode, nrm_after)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    if np.linalg.norm(offset) > 1e-9:
        pcd.translate(offset)

    geoms: List = [pcd]

    bmin, bmax = np.asarray(pcd.points).min(0), np.asarray(pcd.points).max(0)
    extent = np.max(bmax - bmin)

    if show_bbox:
        geoms.append(make_axis_aligned_box(bmin, bmax))

    geoms.append(make_coord_frame(bmin - 0.05 * extent, size=max(0.5, 0.05 * extent)))

    if show_normals and pcd.has_normals():
        nrm = np.asarray(pcd.normals)
        ends = np.asarray(pcd.points) + nrm * (0.05 * extent)
        idx = np.arange(len(pcd.points))
        if len(idx) > 4000:
            idx = np.random.choice(idx, 4000, replace=False)
        line_pts = np.vstack([np.asarray(pcd.points)[idx], ends[idx]])
        lines = np.array([[i, i + len(idx)] for i in range(len(idx))])
        ls = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_pts),
            lines=o3d.utility.Vector2iVector(lines),
        )
        ls.colors = o3d.utility.Vector3dVector(
            np.tile([0.95, 0.50, 0.10], (len(lines), 1))
        )
        geoms.append(ls)

    return geoms, pts, fields


def build_parser(*, with_yml: bool = True) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Standalone PCD visualizer (Open3D)."
    )
    if with_yml:
        p.add_argument(
            "--yml",
            "--yaml",
            dest="yml",
            metavar="PATH",
            default=None,
            help="YAML file whose keys are argparse dest names (underscores); "
                 "CLI flags after YAML override file values.",
        )
    g = p.add_mutually_exclusive_group(required=False)
    g.add_argument("--pcd", type=str, default=None,
                   help="single PCD to view")
    g.add_argument("--compare", type=str, nargs="+", default=None,
                   help="multiple PCDs to view side-by-side")

    p.add_argument("--color", default="height",
                   choices=["height", "intensity", "curvature", "normal", "rgb", "uniform"],
                   help="point coloring mode. 'rgb' uses the file's own "
                        "RGB field (cleaned_scans_*.pcd has it).")
    p.add_argument("--voxel", type=float, default=0.0,
                   help="voxel-downsample size (m); 0 = no downsample")
    p.add_argument("--denoise", action="store_true",
                   help="apply statistical outlier removal")
    p.add_argument("--show-normals", action="store_true",
                   help="overlay short normal hairs")
    p.add_argument("--estimate-normals", action="store_true",
                   help="(re)estimate normals even if PCD has them")
    p.add_argument("--no-bbox", action="store_true",
                   help="hide the axis-aligned bbox wireframe")
    p.add_argument("--gap", type=float, default=2.0,
                   help="extra gap between clouds in --compare mode (in cloud-widths)")
    p.add_argument("--no-vis", action="store_true",
                   help="print stats only, do not open a window")
    p.add_argument("--bg", default="dark", choices=["dark", "light"],
                   help="window background color (default: dark)")
    p.add_argument("--point-size", type=float, default=2.0,
                   help="render point size in pixels (default: 2.0)")
    p.add_argument("--brightness", type=float, default=1.0,
                   help="multiplicative brightness boost on point colors. "
                        ">1 brightens, <1 dims. default 1.0.")
    p.add_argument("--lighting", action="store_true",
                   help="enable Open3D lighting (uses point normals as "
                        "shading; usually makes pure point clouds look "
                        "darker, hence off by default).")
    p.add_argument("--recenter", action="store_true",
                   help="translate the cloud(s) so their joint AABB "
                        "centre lies at the origin before rendering. "
                        "Helps when default Open3D camera clipping "
                        "crops the cloud as you orbit it.")
    p.add_argument("--zoom", type=float, default=0.7,
                   help="initial camera zoom (smaller = farther away). "
                        "Use larger value (e.g. 1.0) to fit tighter; "
                        "use smaller (e.g. 0.4) to back off and avoid "
                        "the near-clip plane biting into the cloud.")
    p.add_argument("--up-axis", default="z",
                   choices=["z", "y", "x", "-y", "-z"],
                   help="rotate the cloud so this axis becomes z-up "
                        "before rendering. Use 'y' for ICL-NUIM "
                        "livingroom.ply / office.ply.")
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
                seq = val
            else:
                seq = [val]
            seq = [str(x) for x in seq]
            if not seq:
                continue
            argv.append(flag)
            argv.extend(seq)
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

    if args.pcd is None and not args.compare:
        raise SystemExit(
            "must pass --pcd or --compare (on the CLI or inside the YAML file)"
        )
    return args


def main(args: Optional[argparse.Namespace] = None):
    if args is None:
        args = parse_args()
    paths = [args.pcd] if args.pcd else list(args.compare)

    for path in paths:
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

    all_geoms: List = []
    cursor_x = 0.0
    for path in paths:
        pts_raw, fields_raw = read_pcd_full(path)
        print_stats(os.path.basename(path), pts_raw, fields_raw)
        del pts_raw, fields_raw

        offset = np.array([cursor_x, 0.0, 0.0])
        geoms, pts, _ = build_geometries_for_one(
            path,
            color_mode=args.color,
            voxel=args.voxel,
            denoise=args.denoise,
            show_normals=args.show_normals,
            estimate_normals=args.estimate_normals,
            offset=offset,
            show_bbox=not args.no_bbox,
        )
        all_geoms.extend(geoms)

        if len(paths) > 1:
            width = float(pts[:, 0].max() - pts[:, 0].min())
            cursor_x += width * (1.0 + args.gap)

    # Rotate to z-up if the input uses a different up-axis (e.g. y-up
    # mesh datasets). Do this BEFORE recenter so the orientation is fixed
    # first, then the camera-friendly recenter happens.
    up = args.up_axis.lower()
    if up != "z":
        if up == "y":
            R = np.array([[1.0, 0.0, 0.0],
                          [0.0, 0.0, -1.0],
                          [0.0, 1.0, 0.0]])
        elif up == "x":
            R = np.array([[0.0, 0.0, 1.0],
                          [0.0, 1.0, 0.0],
                          [-1.0, 0.0, 0.0]])
        elif up == "-y":
            R = np.array([[1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [0.0, -1.0, 0.0]])
        elif up == "-z":
            R = np.diag([1.0, -1.0, -1.0])
        rotated_geoms: List = []
        for g in all_geoms:
            if isinstance(g, o3d.geometry.AxisAlignedBoundingBox):
                # AABBs cannot be rotated; rebuild from the bounds AFTER
                # the corresponding cloud is rotated below. We just
                # transform its corners.
                pts4 = np.asarray(g.get_box_points())
                pts4r = pts4 @ R.T
                bb = o3d.geometry.AxisAlignedBoundingBox(
                    pts4r.min(0), pts4r.max(0)
                )
                bb.color = g.color
                rotated_geoms.append(bb)
            elif hasattr(g, "rotate"):
                g.rotate(R, center=(0.0, 0.0, 0.0))
                rotated_geoms.append(g)
            else:
                rotated_geoms.append(g)
        all_geoms = rotated_geoms
        print(f"[vis] rotated cloud to z-up (input was '{up}-up')")

    if args.no_vis:
        print("\n[--no-vis] skipping window.")
        return

    title = "PCD viewer: " + ", ".join(os.path.basename(p) for p in paths)

    # Optional global brightness boost (applied AFTER colorize, before render)
    if abs(args.brightness - 1.0) > 1e-6:
        bf = float(args.brightness)
        for g in all_geoms:
            if isinstance(g, o3d.geometry.PointCloud) and g.has_colors():
                cols = np.asarray(g.colors) * bf
                g.colors = o3d.utility.Vector3dVector(np.clip(cols, 0.0, 1.0))

    # Optional: recenter joint AABB to the origin BEFORE create_window so
    # the camera (which sits a fixed distance from origin by default) does
    # not bite into the cloud with its near clip plane.
    if args.recenter:
        all_pts = []
        for g in all_geoms:
            if isinstance(g, o3d.geometry.PointCloud):
                all_pts.append(np.asarray(g.points))
        if all_pts:
            P = np.concatenate(all_pts, axis=0)
            center = (P.min(0) + P.max(0)) * 0.5
            for g in all_geoms:
                if hasattr(g, "translate"):
                    g.translate(-center)
            print(f"[vis] recentered scene by {-center.round(3)}")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=900)
    for g in all_geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    if args.bg == "dark":
        opt.background_color = np.array([0.06, 0.07, 0.10])
        # invert the bbox color from black to light grey so it stays visible
        for g in all_geoms:
            if isinstance(g, o3d.geometry.AxisAlignedBoundingBox):
                g.color = (0.85, 0.85, 0.90)
                vis.update_geometry(g)
    else:
        opt.background_color = np.array([0.97, 0.97, 0.99])
    opt.point_size = float(args.point_size)
    # Lighting OFF by default: when a cloud carries normals Open3D treats
    # them as shading normals, which dims any point whose normal faces
    # away from the (front) light. For colored clouds this nearly always
    # makes the result look darker than intended.
    opt.light_on = bool(args.lighting)

    # Pull the camera back so the near-clip plane doesn't slice into the
    # cloud; smaller zoom = farther.
    vc = vis.get_view_control()
    try:
        vc.set_zoom(float(args.zoom))
    except Exception:
        pass

    print("\n[vis] press H in the window for keyboard shortcuts; Q to close.")
    print("[vis] tip: if the cloud gets sliced as you orbit, press 'J' to "
          "switch to orthographic projection, or rerun with a smaller "
          "--zoom value (e.g. 0.4).")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
