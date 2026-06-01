# MW-PRM — Multi-Modal PRM (MMPRM)

Probabilistic Roadmap (PRM) path planning with **multiple locomotion modes**
(ground walking plus aerial / climbing). This repository provides two
interchangeable scene backends:

- **Analytic** — Convex polyhedron obstacles from `utils/parameters*.py`
  (legacy `main.py` + Matplotlib).
- **Point cloud** — `.pcd` / `.ply` scans with KD-tree collision checking and a
  2.5D heightmap barrier (`main_pcd.py` + Open3D).

The same multi-modal A\* search and B-spline smoothing run in both settings;
only the collision checker is swapped via dependency injection.

**Stack:** Python 3.9+ (3.10 in `environment.yml`), NumPy, SciPy, Open3D,
Matplotlib, NetworkX, scikit-learn, PyYAML (optional `rerun-sdk`).

---

## Repository layout

```text
MW-PRM/
├── main.py                    # Analytic obstacles + Matplotlib demo
├── main_pcd.py                # Point-cloud / Open3D demo (CLI + optional YAML)
├── synth_pcd.py               # Synthetic .pcd from analytic obstacles
├── visualize_pcd.py           # Standalone point cloud viewer
├── plot_csv.py                # Plot paths from CSV
├── convert_mesh_to_zup.py     # Mesh axis utilities (e.g. ICL-NUIM → z-up)
├── environment.yml            # Conda environment (`mmprm`)
├── run_from_yml.sh            # Example: run `main_pcd.py` from a YAML preset
├── run_visualize_from_yml.sh
├── yml/
│   ├── pcd/                   # Planner YAML presets
│   └── visualize/             # Visualization presets
├── pointcloude/               # Sample .pcd inputs (dirname spelling is historical)
├── datasets/                  # Example meshes / scenes (e.g. Stanford Bunny)
└── utils/
    ├── env_pointcloud.py      # PCD pipeline, KD-tree + heightmap checker
    ├── graph.py               # Env-agnostic PRM (injected collision checker)
    ├── path_planning.py       # Multi-modal A\* and B-spline smoothing
    ├── parameters.py          # Large analytic scene
    ├── parameters_test.py     # Small analytic test scene
    └── plotting.py            # Matplotlib helpers (legacy)
```

---

## Environment setup

### Option A — Conda from `environment.yml` (recommended)

```bash
conda env create -f environment.yml
conda activate mmprm
```

If conda channel access is flaky, retry with a mirror:

```bash
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
conda config --set show_channel_urls yes
conda env create -f environment.yml
```

### Option B — Plain pip into any Python ≥ 3.9

```bash
pip install -U \
    numpy scipy matplotlib pandas networkx tqdm scikit-learn \
    open3d==0.18.0
# optional, for the alternative renderer
pip install rerun-sdk==0.17.0 PyYAML
```

Use a domestic PyPI mirror with `-i https://pypi.tuna.tsinghua.edu.cn/simple`
if needed.

### Option C — Reuse an existing env

Any environment that already has `open3d>=0.18` plus the standard
scientific stack (`numpy`, `scipy`, `matplotlib`, `pandas`, `networkx`,
`scikit-learn`) is fine. Activate it and skip to **Quick start**.

### Dependencies summary

| Package | Version | Used by |
|---|---|---|
| `python` | ≥ 3.9 (3.10 in `environment.yml`) | everything |
| `numpy` | 1.26.x | everything |
| `scipy` | 1.13.x | KD-tree, B-spline |
| `open3d` | 0.18.0 (or newer) | PCD I/O, KD-tree, 3D viewer |
| `matplotlib` | 3.8.x | legacy `main.py` viewer |
| `pandas` | 2.2.x | `plot_csv.py` |
| `networkx` | 3.2.x | available for graph utilities |
| `scikit-learn` | latest | downsampling, optional |
| `PyYAML` | ≥ 6.0 | `main_pcd.py --yml`, YAML presets |
| `tqdm` | latest | progress bars |
| `rerun-sdk` | 0.17.0 (optional) | alternative renderer |

---

## Quick start

### Legacy analytic-obstacle demo

```bash
python main.py
```

Pops a matplotlib window with the obstacles, PRM samples and the planned
path. Parameters are taken from `utils/parameters_test.py::get_parameters`.

### Point-cloud demo (Open3D)

Run from a YAML preset:

```bash
./run_from_yml.sh
# or: python main_pcd.py --yml yml/pcd/synth_parameters_test_connect_obstacles.yml
```

A real scan:

```bash
python main_pcd.py --pcd pointcloude/scans_e.pcd
```

A synthetic cloud built from the analytic test obstacles, with a known
start / goal:

```bash
# 1) generate the synthetic .pcd  (only needed once)
python synth_pcd.py --params parameters_test --layer real_obstacles \
    --out pointcloude/synth_parameters_test_real_obstacles.pcd

# 2) run the planner
python main_pcd.py \
    --pcd pointcloude/synth_parameters_test_real_obstacles.pcd \
    --voxel 0.3 --r-safe 0.45 --rmax 4.0 \
    --num-ground 1200 --num-surface 600 --num-aerial 800 \
    --max-surface-z 15 \
    --start -24 -5 0 --end 15.5 6.5 11 \
    --wg 1 --wf 5 --e-factor 0.1 \
    --smoothing 5 --lift-epsilon 1.0
```

Press `H` in the Open3D window for keyboard shortcuts; `Q` to close.

### Synthetic PCD generator

Render the analytic obstacles into a binary PCD (with intensity, normals,
curvature) so the PCD pipeline can be unit-tested:

```bash
# big scene from parameters.py
python synth_pcd.py --params parameters --out pointcloude/synth_params.pcd

# small test scene, expanded obstacles
python synth_pcd.py --params parameters_test --layer expand_obstacles \
    --out pointcloude/synth_test_expand.pcd

# add some noise + view immediately
python synth_pcd.py --params parameters --density 80 --noise 0.05 --vis
```

### Point cloud inspector

```bash
# default (height) coloring
python visualize_pcd.py --pcd pointcloude/scans_e.pcd

# color by intensity / show normals as hairs
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --color intensity
python visualize_pcd.py --pcd pointcloude/scans_e.pcd --color normal --show-normals

# compare three scans side-by-side
python visualize_pcd.py --compare pointcloude/scans_e.pcd \
                                  pointcloude/scans_q.pcd \
                                  pointcloude/scans_w.pcd

# downsample + denoise + print stats only
python visualize_pcd.py --pcd pointcloude/scans_e.pcd \
    --voxel 0.1 --denoise --no-vis
```

---

## `main_pcd.py` CLI reference

### Point-cloud preprocessing

| Flag | Default | Meaning |
|---|---|---|
| `--pcd PATH` | `pointcloude/scans_e.pcd` | input `.pcd` file |
| `--voxel FLOAT` | `0.2` | voxel down-sample size (m) |
| `--r-safe FLOAT` | `0.4` | collision safety radius (m) |
| `--ground-thresh FLOAT` | `0.15` | RANSAC ground-plane distance threshold (m) |
| `--ground-clearance FLOAT` | `0.3` | exclude `\|z\| < this` from the obstacle cloud |
| `--max-surface-z FLOAT` | `18.0` | cut walkable surfaces above this height |
| `--max-slope-deg FLOAT` | `50.0` | max slope considered walkable |
| `--surface-lift FLOAT` | `None` | optional lift for walkable-surface samples; by default samples stay directly on the surface and that surface is carved out of the 3D inflated obstacle cloud |
| `--walkable-edge-margin FLOAT` | `r_safe` | keep this XY boundary band of each walkable surface inflated, so samples are only placed on the safe interior |

### PRM graph

| Flag | Default | Meaning |
|---|---|---|
| `--rmax FLOAT` | `4.0` | PRM connection radius (m) |
| `--num-ground INT` | `600` | # ground PRM nodes |
| `--num-surface INT` | `400` | # walkable-surface PRM nodes |
| `--num-aerial INT` | `400` | # aerial PRM nodes |
| `--seed INT` | `0` | RNG seed |
| `--start X Y Z` | auto | start point; if omitted, picked from samples |
| `--end X Y Z` | auto | goal point; if omitted, picked from samples |

### Multi-modal A\* cost model

| Flag | Default | Meaning |
|---|---|---|
| `--wg FLOAT` | `1.0` | ground-walking cost per metre |
| `--wf FLOAT` | `5.0` | aerial / climbing cost per metre |
| `--e-factor FLOAT` | `0.1` | mode-switch penalty as a fraction of `wg` |

### Path smoothing

| Flag | Default | Meaning |
|---|---|---|
| `--smoothing FLOAT` | `5.0` | B-spline smoothing factor |
| `--spline-degree INT` | `3` | B-spline degree (2..5) |
| `--lift-epsilon FLOAT` | `1.0` | vertical lift (m) of virtual control points near take-off / landing on `wf` segments |

### Visualization

| Flag | Default | Meaning |
|---|---|---|
| `--no-vis` | off | run the planner but skip the Open3D window (CI-friendly) |
| `--show-prm` | off | overlay PRM ground edges |
| `--no-samples` | off | hide PRM sample points |
| `--bg {dark,light}` | `dark` | window background |
| `--dim-raw FLOAT` | `0.7` | raw cloud dimming factor (1=full color) |
| `--path-radius FLOAT` | auto | path-tube radius (m); auto = 0.5 % of scene extent |
| `--marker-radius FLOAT` | auto | start / goal sphere radius (m); auto = 1.5 % of extent |
| `--point-size FLOAT` | `2.0` | raw cloud point size (px) |

---

## Tuning the planner

Quick rules of thumb:

* `rmax ≈ 4–6 × voxel`, **and** at least 2× the average node spacing.
* `r_safe ≥ 1.5 × voxel`, otherwise voxel quantisation alone can flag
  edges as in-collision.
* Sample counts scale with the area / volume that needs to be covered.
  Increase them when the planner reports `no path found`.
* `wf / wg` is the "1 m of flight equals N m of walking" ratio.
  Increase `wf` (or `e-factor`) to favour walking; decrease to fly more.
* `lift-epsilon` controls how vertical take-off / landing curves look,
  without changing the planned (collision-checked) waypoints.

If the planner cannot find a path, the most useful first debug step is
`--show-prm`: it overlays the ground edges so you can spot disconnected
islands.

---

## Design notes

### Environment-agnostic graph builder

`utils/graph.py::connect_nearby_nodes` only needs a callable with the
signature `collision_checker(p1, p2) -> bool`. Two adapters are provided:

* `make_polyhedron_collision_checker(params)` — analytic convex polyhedra
  (used by the legacy `main.py`).
* `utils.env_pointcloud.PointCloudCollisionChecker` — point-cloud KD-tree
  with 2.5D heightmap barrier (used by `main_pcd.py`).

### 2.5D heightmap barrier

In addition to the standard 3D KD-tree distance check, the point-cloud
checker keeps a per-xy-cell maximum-z map ("local roof height"). A query
point `(x, y, z)` counts as in-collision when `z < heightmap(x, y) − r_safe`,
i.e. it sits *under* an obstacle's local roof. This catches two
otherwise-undetected cases:

* ground edges that diagonally cross an obstacle's xy footprint at z=0
  after `--ground-clearance` has removed the lower wall band;
* aerial samples that fall *inside* a closed obstacle volume.

### Multi-modal cost preserved

`generate_nodes_pointcloud` returns the samples and an `is_surface`
boolean mask. `connect_nearby_nodes` uses it to assign edge weights
exactly like the original implementation:

* ground ↔ ground: `wg`
* surface ↔ surface: `wg`
* aerial ↔ * : `wf`
* mixed: `[wf, e]` with mode-switch penalty `e = wg · e_factor`

### A\* untouched

`utils/path_planning.py::astar` is unchanged from the analytic version;
the multi-modal heuristic and B-spline smoother work identically across
both backends.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `no path found` | PRM disconnected | larger `--rmax`, more samples, `--show-prm` to visualise |
| start / goal stuck inside a wall | snapped to the wrong free node | pass an explicit `--start / --end` away from obstacles |
| path visibly clips an obstacle | scan noise has filled the obstacle interior | increase `--ground-clearance`; verify with `visualize_pcd.py` |
| `_tkinter.TclError` on a headless box | matplotlib trying to open a GUI | run `main.py` only in a desktop session, or use `main_pcd.py` with `--no-vis` |
| Open3D window is empty / black | bad GL drivers | upgrade graphics drivers, or run `--no-vis` and inspect the textual output |
| `ImportError: open3d` | env not activated | `conda activate mmprm` (or your env), or reinstall per **Environment setup** |

---

## Reproducing the multi-modal demo

The combination below reproduces the figure used during development
(takes ~20 s on a recent CPU):

```bash
python synth_pcd.py --params parameters_test --layer real_obstacles \
    --out pointcloude/synth_parameters_test_real_obstacles.pcd

python main_pcd.py \
    --pcd pointcloude/synth_parameters_test_real_obstacles.pcd \
    --voxel 0.3 --r-safe 0.45 --rmax 4.0 \
    --num-ground 1200 --num-surface 600 --num-aerial 800 \
    --max-surface-z 15 \
    --start -24 -5 0 --end 15.5 6.5 11 \
    --wg 1 --wf 5 --e-factor 0.1 \
    --smoothing 5 --lift-epsilon 1.0 \
    --bg dark --dim-raw 0.55 --no-samples
```

Expected output:

```
[1/5] loading pointcloude/synth_parameters_test_real_obstacles.pcd ...
      raw points    :    42192
      ground points :    12347
      non-ground    :    29174
[2/5] sampling PRM nodes ...
      ground / surface / aerial = 1200 / 600 / 800
[3/5] connecting PRM edges (R=4.0, N=2602) ...
      total edges : ~48000
[4/5] running multi-modal A* ...
      path waypoints : ~22-25
      path length    : ~63 m
[5/5] preparing visualization ...
```
