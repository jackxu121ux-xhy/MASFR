"""Environment-agnostic PRM graph utilities for MMPRM.

The original implementation hard-coded an analytic-polyhedron collision
check inside ``connect_nearby_nodes``. This module exposes the same
algorithms but takes any ``CollisionChecker`` callable so we can plug in:

  * ``make_polyhedron_collision_checker(obstacles)`` for legacy scenes
  * ``utils.env_pointcloud.PointCloudCollisionChecker`` for scanned scenes

Public API:

  * ``CollisionChecker``                      type alias
  * ``heuristic``                             Euclidean distance
  * ``make_polyhedron_collision_checker``     legacy adapter
  * ``generate_nodes_pointcloud``             new sampler (point cloud)
  * ``generate_nodes``                        legacy sampler (polyhedra)
  * ``connect_nearby_nodes``                  PRM edge builder w/ DI
"""

from __future__ import annotations

from collections import defaultdict
from typing import Callable, Dict, List, Optional, Set, Tuple

import numpy as np
from scipy.spatial import ConvexHull, cKDTree


CollisionChecker = Callable[[np.ndarray, np.ndarray], bool]


# ---------------------------------------------------------------------------
# Distance helper
# ---------------------------------------------------------------------------

def heuristic(a, b) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return float(np.linalg.norm(b - a))


# ---------------------------------------------------------------------------
# Node generation -- point cloud version
# ---------------------------------------------------------------------------

def generate_nodes_pointcloud(
    params: dict,
    ground_pts: np.ndarray,
    surface_pts: np.ndarray,
    aerial_pts: np.ndarray,
) -> Tuple[np.ndarray, int, int, np.ndarray]:
    """Concatenate samples + start/goal and bookkeep a ``is_surface`` mask.

    Returns
    -------
    samples : (N, 3) float
    start_idx, end_idx : int
    is_surface : (N,) bool   -- True for the ``surface_pts`` block (walkable
                                elevated / non-floor band in split mode, or
                                all unified walk nodes). Used by edge weights.
    """
    start = np.asarray(params["start"], dtype=np.float64).reshape(1, 3)
    end = np.asarray(params["end"], dtype=np.float64).reshape(1, 3)

    ground_pts = np.asarray(ground_pts, dtype=np.float64).reshape(-1, 3)
    surface_pts = np.asarray(surface_pts, dtype=np.float64).reshape(-1, 3)
    aerial_pts = np.asarray(aerial_pts, dtype=np.float64).reshape(-1, 3)

    samples = np.vstack([ground_pts, surface_pts, aerial_pts, start, end])

    n_g = len(ground_pts)
    n_s = len(surface_pts)
    n_a = len(aerial_pts)

    is_surface = np.zeros(len(samples), dtype=bool)
    is_surface[n_g : n_g + n_s] = True

    start_idx = n_g + n_s + n_a
    end_idx = start_idx + 1
    return samples, start_idx, end_idx, is_surface


# ---------------------------------------------------------------------------
# Node generation -- legacy polyhedron version (kept for backward compat)
# ---------------------------------------------------------------------------

def generate_nodes(
    params,
    obstacles_faces,
    total_points,
    area_threshold,
    max_height,
    threshold_angle,
):
    """Original sampler used by ``main.py`` (analytic polyhedra)."""
    num_samples = params["num_samples"]
    num_node_g = int(params["ground_ratio"] * num_samples)
    num_node_f = num_samples - num_node_g

    node_g = np.random.uniform(
        low=[params["xlim"][0], params["ylim"][0]],
        high=[params["xlim"][1], params["ylim"][1]],
        size=(num_node_g, 2),
    )
    node_g = np.hstack((node_g, np.zeros((num_node_g, 1))))
    node_g = np.vstack([node_g, params["start"]])
    start_idx = len(node_g) - 1

    obstacles_surface_points = generate_points_on_prism(
        obstacles_faces, total_points, area_threshold, max_height, threshold_angle
    )

    node_f = np.random.uniform(
        low=[params["xlim"][0], params["ylim"][0], params["zlim"][0]],
        high=[params["xlim"][1], params["ylim"][1], params["zlim"][1]],
        size=(num_node_f, 3),
    )
    node_f = np.vstack([node_f, params["end"]])
    end_idx = len(node_g) + len(node_f) - 1

    samples = np.vstack((node_g, node_f, obstacles_surface_points))
    obstacles_surface_points = np.vstack([obstacles_surface_points, params["end"]])
    return samples, start_idx, end_idx, obstacles_surface_points


def generate_points_on_prism(
    obstacles_faces, total_points, area_threshold, height_threshold, threshold_angle
):
    def calculate_normal(points):
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        n = np.cross(v1, v2)
        return n / np.linalg.norm(n)

    def is_valid_face(face_points):
        face_points = np.array(face_points)
        if np.allclose(face_points[:, 2], 0):
            return False
        if np.max(face_points[:, 2]) > height_threshold:
            return False
        n = calculate_normal(face_points)
        vertical = np.array([0, 0, 1])
        angle = np.degrees(np.arccos(np.clip(np.dot(n, vertical), -1, 1)))
        return angle < threshold_angle

    def polygon_area(points):
        x = points[:, 0]
        y = points[:, 1]
        return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    def triangulate_quad(points):
        a1 = polygon_area(points[[0, 1, 2]]) + polygon_area(points[[0, 2, 3]])
        a2 = polygon_area(points[[0, 1, 3]]) + polygon_area(points[[1, 2, 3]])
        return [[0, 1, 2], [0, 2, 3]] if a1 <= a2 else [[0, 1, 3], [1, 2, 3]]

    def gen_tri(A, B, C, n):
        out = []
        for _ in range(n):
            r1, r2 = np.random.rand(2)
            if r1 + r2 > 1:
                r1, r2 = 1 - r1, 1 - r2
            out.append(A + r1 * (B - A) + r2 * (C - A))
        return np.array(out)

    valid_faces, face_areas = [], []
    for faces_points in obstacles_faces:
        for face_points in faces_points:
            if is_valid_face(face_points):
                area = polygon_area(face_points)
                if area > area_threshold:
                    valid_faces.append(face_points)
                    face_areas.append(area)

    if not valid_faces:
        return np.empty((0, 3))

    total_area = sum(face_areas)
    points = []
    for face, area in zip(valid_faces, face_areas):
        triangles = triangulate_quad(face) if len(face) == 4 else [list(range(len(face)))]
        tri_areas = [polygon_area(face[t]) for t in triangles]
        tri_n = [int(total_points * a / total_area) for a in tri_areas]
        for t, n in zip(triangles, tri_n):
            A, B, C = face[t[0]], face[t[1]], face[t[2]]
            points.extend(gen_tri(A, B, C, n))

    return np.array(points)[:total_points]


# ---------------------------------------------------------------------------
# Edge construction with dependency-injected collision checker
# ---------------------------------------------------------------------------

def connect_nearby_nodes(
    samples: np.ndarray,
    params: dict,
    collision_checker: CollisionChecker,
    is_surface: Optional[np.ndarray] = None,
    *,
    obstacles=None,
    obstacle_surface_points=None,
) -> Tuple[Dict[Tuple, Set[Tuple]], Dict[frozenset, object]]:
    """Build PRM edges within ``R_max`` using a pluggable collision checker.

    ``weights`` maps ``frozenset({p_i,p_j})`` to tagged multimodal tuples:

      * ``("wg", c)``  — walking / ground-style edge (cost ``d*c``).
      * ``("wf", c)``  — pure aerial (cost ``d*c``).
      * ``("sw", c, p)`` — mode-switch (cost ``d*c + p``, with ``p=wg·e_factor``).

    Plain floats would make ``wf==wg`` configurations ambiguous downstream.

    The two ``obstacles`` / ``obstacle_surface_points`` keyword args are
    *only* there for backward compatibility with the legacy call site in
    ``main.py``::

        edges, w = connect_nearby_nodes(samples, params, expand_obstacles, surf)

    Modern callers should pass a ``collision_checker`` callable and an
    ``is_surface`` boolean mask instead.
    """
    if not callable(collision_checker):
        legacy_obstacles = collision_checker
        legacy_surface_points = is_surface
        collision_checker = make_polyhedron_collision_checker(legacy_obstacles)
        is_surface = _surface_mask_from_points(samples, legacy_surface_points)

    if is_surface is None:
        is_surface = np.zeros(len(samples), dtype=bool)
    is_surface = np.asarray(is_surface, dtype=bool)

    samples = np.asarray(samples, dtype=np.float64)

    wg = float(params["wg"])
    wf = float(params["wf"])
    e_pen = wg * float(params["e_factor"])
    R_max = params["R_max"]

    edges: Dict[Tuple, Set[Tuple]] = defaultdict(set)
    weights: Dict[frozenset, object] = {}

    tree = cKDTree(samples)
    pair_set: Set[Tuple[int, int]] = set()
    for i, neigh in enumerate(tree.query_ball_point(samples, r=R_max)):
        for j in neigh:
            if j <= i:
                continue
            pair_set.add((i, j))

    for i, j in pair_set:
        pi = samples[i]
        pj = samples[j]
        if collision_checker(pi, pj):
            continue

        ti = tuple(pi.tolist())
        tj = tuple(pj.tolist())
        edges[ti].add(tj)
        edges[tj].add(ti)
        edge_key = frozenset([ti, tj])

        zi, zj = pi[2], pj[2]
        si, sj = bool(is_surface[i]), bool(is_surface[j])

        if zi <= 1e-6 and zj <= 1e-6:
            weights[edge_key] = ("wg", wg)
        elif zi > 1e-6 and zj > 1e-6:
            if si and sj:
                weights[edge_key] = ("wg", wg)
            elif si != sj:
                weights[edge_key] = ("sw", wf, e_pen)
            else:
                weights[edge_key] = ("wf", wf)
        else:
            weights[edge_key] = ("sw", wf, e_pen)

    return edges, weights


# ---------------------------------------------------------------------------
# Legacy polyhedron collision checker (adapter)
# ---------------------------------------------------------------------------

def make_polyhedron_collision_checker(
    obstacles: List, angle_threshold: float = 50.0
) -> CollisionChecker:
    """Wrap the original analytic ray-vs-polyhedron test as a callable."""

    def checker(p1: np.ndarray, p2: np.ndarray) -> bool:
        return _check_collision_polyhedra(p1, p2, obstacles, angle_threshold)

    return checker


def _surface_mask_from_points(
    samples: np.ndarray, surface_points: Optional[np.ndarray]
) -> np.ndarray:
    """Best-effort mask: True iff a sample matches any surface_point.

    Uses a KD-tree with a tiny tolerance so we don't depend on hashing
    floating-point tuples.
    """
    n = len(samples)
    if surface_points is None or len(surface_points) == 0:
        return np.zeros(n, dtype=bool)

    surface_points = np.asarray(surface_points, dtype=np.float64).reshape(-1, 3)
    tree = cKDTree(surface_points)
    d, _ = tree.query(samples, k=1)
    return d < 1e-6


# ---------------------------------------------------------------------------
# Internal: legacy polyhedron collision (preserved verbatim, just relocated)
# ---------------------------------------------------------------------------

def _is_inside_convex_polyhedron(point, obstacle):
    points = np.unique(np.array([p for face in obstacle for p in face]), axis=0)
    hull = ConvexHull(points)
    return all(np.dot(eq[:-1], point) + eq[-1] < 0 for eq in hull.equations)


def _check_collision_polyhedra(line_start, line_end, obstacles, angle_threshold=50):
    EPSILON = 1e-6

    def ray_plane_intersection(ro, rd, plane):
        plane = np.array(plane)
        v1 = plane[1] - plane[0]
        v2 = plane[2] - plane[0]
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)

        start_on_plane = np.abs(np.dot(ro - plane[0], normal)) < EPSILON
        end_on_plane = np.abs(np.dot((ro + rd) - plane[0], normal)) < EPSILON
        if start_on_plane and end_on_plane:
            return ro

        denom = np.dot(normal, rd)
        if abs(denom) < EPSILON:
            if start_on_plane:
                return ro
            elif end_on_plane:
                return ro + rd
            return None

        t = np.dot(plane[0] - ro, normal) / denom
        if t < 0 or t > 1:
            return None
        return ro + rd * t

    def is_on_face_projection(point, face):
        if len(face) < 3:
            return False
        v0 = face[0]
        v1 = face[1] - v0
        v2 = face[2] - v0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < EPSILON:
            return False
        normal = normal / norm

        axis = np.argmax(np.abs(normal))
        u = (axis + 1) % 3
        v = (axis + 2) % 3
        proj = np.array(
            [[u == 0, u == 1, u == 2], [v == 0, v == 1, v == 2]], dtype=float
        )
        poly2d = [proj @ vertex for vertex in face]
        px, py = proj @ point
        wn = 0
        for i in range(len(poly2d)):
            p1 = poly2d[i]
            p2 = poly2d[(i + 1) % len(poly2d)]
            if abs(p2[1] - p1[1]) < EPSILON:
                continue
            t = (py - p1[1]) / (p2[1] - p1[1])
            ix = p1[0] + t * (p2[0] - p1[0])
            if p1[1] <= py:
                if p2[1] > py and ix > px:
                    wn += 1
            else:
                if p2[1] <= py and ix > px:
                    wn -= 1
        return wn != 0

    def calc_angle(normal):
        ref = np.array([0, 0, 1])
        cos_t = np.dot(normal, ref) / (np.linalg.norm(normal) * np.linalg.norm(ref))
        return np.arccos(np.clip(cos_t, -1, 1)) * 180 / np.pi

    line_start = np.array(line_start, dtype=np.float64)
    line_end = np.array(line_end, dtype=np.float64)
    direction = line_end - line_start

    for obstacle in obstacles:
        if _is_inside_convex_polyhedron(line_start, obstacle) or _is_inside_convex_polyhedron(
            line_end, obstacle
        ):
            return True

    for obstacle in obstacles:
        for face in obstacle:
            face = np.array(face)
            v1 = face[1] - face[0]
            v2 = face[2] - face[0]
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)
            angle = calc_angle(normal)
            inter = ray_plane_intersection(line_start, direction, face)
            if inter is None:
                continue
            if not is_on_face_projection(inter, face):
                continue
            if np.array_equal(inter, line_start) or np.array_equal(inter, line_end):
                if angle <= angle_threshold:
                    if inter[2] == 0:
                        return True
                    continue
                return True
            return True
    return False


def check_collision(line_start, line_end, obstacles, angle_threshold=50):
    """Backward-compat alias for the legacy polyhedron collision test."""
    return _check_collision_polyhedra(line_start, line_end, obstacles, angle_threshold)
