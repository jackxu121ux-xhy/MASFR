# MASFR: Multimodal Adhesion, Sliding, and Flying Robot

This repository collects replication materials for MASFR, a bioinspired multimodal robot that combines aerial flight with adhesion-assisted surface crawling. MASFR uses distributed micro-setae suction feet inspired by net-winged midge larvae, pressure-regulated adhesion-sliding gaits, and a multi-weighted probabilistic roadmap planner (MW-PRM) for structured multimodal path generation.

The repository is organized to support manuscript review and replication. It includes the MW-PRM planner code and example point-cloud scenes, and reserves folders for raw experimental data, controller parameters, representative CAD/STL files, and annotated videos of successful and representative failed trials.

## Manuscript Summary

Flying-perching robots can save energy by attaching to surfaces, but many existing systems must repeatedly take off, realign, and perch when they need to change position after landing. MASFR addresses this limitation by combining flight with controllable adhesion-sliding on surfaces.

The robot is built around four articulated adhesion-sliding legs mounted on a quadrotor platform. Each leg uses a Distributed Micro-setae Sucker (DMS), whose pressure-regulated contact state can switch between high-adhesion and low-friction sliding. In the manuscript experiments, the DMS provides a friction modulation ratio of 407.5%, reducing wet-state shear stress from 21.6 kPa in adhesion mode to 5.3 kPa in sliding mode. The robot demonstrates forward, lateral, and turning gaits through alternating adhesion and sliding phases.

MASFR also includes a lightweight closed-loop crawling controller that uses heading and lateral path errors to generate turning and forward commands for surface trajectory tracking. In robot-level tests, MASFR crawled on a 40 degree inclined surface against 1.5 m/s flow while carrying a 2 kg payload. The reported climbing power consumption was 21.7 W, about 2.3% of flight power under the tested condition.

For mission-level autonomy in known or pre-mapped environments, the project uses MW-PRM. The planner samples aerial, ground, and adhesive-surface nodes from geometric or point-cloud maps, constructs feasible flight, crawling, and adhesive-surface edges, and runs weighted A* over a hybrid graph. The cost model balances energy, traversal time, and mode-switching penalties. Indoor experiments show optimized multimodal trajectories that reduce energy consumption by 31.7% and improve positioning precision by 58.5% compared with single-mode flight.

Outdoor demonstrations on frozen lakeshores, rough pavement, inclined glass roofs, and narrow beam structures validate mechanical feasibility and environmental adaptability. These outdoor tests were pilot-controlled demonstrations rather than online SLAM-based autonomous navigation.

## Repository Layout

```text
MASFR/
├── algorithms/
│   └── mw_prm/                         # MW-PRM planner code and examples
├── data/
│   └── raw/                            # Raw experimental data placeholders
│       ├── adhesion/
│       ├── friction/
│       ├── trajectory_tracking/
│       ├── energy_consumption/
│       └── repeated_trials/
├── controller_parameters/              # Controller and execution parameters
│   ├── crawling/
│   ├── flight/
│   └── multimodal_switching/
├── cad/                                # Representative CAD/STL placeholders
│   ├── dms_foot/
│   └── leg_module/
└── videos/
    └── annotated/
        ├── successes/
        └── failures/
```

## MW-PRM Algorithm Module

The planner code is provided in `algorithms/mw_prm/`. It supports two environment backends:

- analytic obstacle scenes from Python parameter files;
- point-cloud scenes from `.pcd` or `.ply` files using Open3D, KD-tree collision checking, and 2.5D height-map constraints.

The planner builds a multimodal graph with ground/surface crawling nodes and aerial nodes. Edges are assigned locomotion-mode costs and switching penalties, then searched with A* to produce mode-labeled paths.

### Environment Setup

From the repository root:

```bash
cd algorithms/mw_prm
conda env create -f environment.yml
conda activate mmprm
```

If Conda is unavailable, use Python 3.9+ and install the scientific stack manually. On Python 3.12, `open3d==0.18.0` may not provide wheels on Windows; `open3d==0.19.0` was used successfully for the local smoke test.

### Smoke Test

```bash
cd algorithms/mw_prm
python main_pcd.py --yml yml/pcd/synth_parameters_obstacles.yml --no-vis
```

A successful run loads the synthetic point cloud, samples PRM nodes, connects feasible edges, runs multimodal A*, reports path waypoints and path length, then exits without opening the Open3D viewer because `--no-vis` is enabled.

## Data and Materials Availability

The manuscript states that the main data are available in the main text or supplementary materials. This repository is structured to deposit the raw data supporting adhesion, friction, trajectory-tracking, energy-consumption, and robot-level repeated-trial results. The folders currently reserve the expected locations for those files and include short notes describing the intended contents.

The repository also reserves locations for controller parameters, representative CAD/STL files of the DMS foot and leg module, and annotated videos of successful and representative failed trials. Additional hardware files may be made available from the corresponding authors upon reasonable request.

## Current Scope and Limitations

The included MW-PRM implementation assumes known or pre-mapped static environments and generates mode-labeled trajectories before execution. It does not yet implement online SLAM or dynamic global replanning. The suction system described in the manuscript currently uses calibrated open-loop pressure regulation; embedded pressure or tactile feedback is a planned extension. Dynamic perching tests in the manuscript showed reliable landing on planar inclined acrylic surfaces up to about 40 degrees under the tested landing conditions.

## Citation

Citation information will be added after publication or preprint release. For questions about the manuscript or hardware materials, please contact the corresponding authors listed in the manuscript.
