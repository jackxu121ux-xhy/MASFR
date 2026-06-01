"""Point-cloud environment utilities for MMPRM.

This module replaces the analytic polyhedron representation in
``utils/parameters_test.py`` with a real LiDAR / depth point cloud:

    pcd  -->  preprocess  -->  ground / non-ground split
                                 |
                                 +-->  KD-tree collision checker
                                 +-->  ground samples (z ~ 0)
                                 +-->  surface samples (walkable, z>0)

The public surface is intentionally small so that ``utils/graph.py`` can
remain completely environment-agnostic and only depends on:

    * ``CollisionChecker`` callable: ``(p1, p2) -> bool``
    * three numpy arrays of ``(N, 3)`` candidate nodes:
        ground_pts, surface_pts, aerial_pts
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import os as _os

# Linux/Wayland: default GLFW to X11 so Open3D often uses XWayland with a
# working GL stack (native Wayland frequently hits "Failed to initialize GLEW").
# Set GLFW_PLATFORM=wayland in the environment to force native Wayland instead.
_os.environ.setdefault("GLFW_PLATFORM", "x11")

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise ImportError(
        "open3d is required for utils.env_pointcloud. "
        "Install it with `pip install open3d==0.18.0`."
    ) from e


CollisionChecker = Callable[[np.ndarray, np.ndarray], bool]


# ---------------------------------------------------------------------------
# I/O & preprocessing
# ---------------------------------------------------------------------------

def load_pointcloud(
    path: str,
    voxel_size: float = 0.2,
    # Statistical outlier removal (SOR): number of nearest neighbours used
    # to estimate each point's local mean distance. Larger values make the
    # noise estimate smoother but can remove small isolated details.
    sor_neighbors: int = 20,
    # SOR threshold in standard deviations. Smaller values remove more
    # points aggressively; larger values preserve more sparse structure.
    sor_std_ratio: float = 2.0,
) -> o3d.geometry.PointCloud:
    """Load a PCD/PLY/XYZ file and apply standard preprocessing.

    Steps: read -> statistical outlier removal -> voxel downsample ->
    estimate normals (if missing).
    """
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise ValueError(f"Empty point cloud: {path}")

    if sor_neighbors and sor_neighbors > 0:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=sor_neighbors, std_ratio=sor_std_ratio
        )

    if voxel_size and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(0.4, 3.0 * (voxel_size or 0.2)), max_nn=30
            )
        )
        pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

    return pcd


def crop_pcd_below_z(
    pcd: o3d.geometry.PointCloud,
    z_keep_max: float,
) -> o3d.geometry.PointCloud:
    """Return a cloud with only points whose z-coordinate is <= ``z_keep_max``.

    Intended for *visualization* (e.g. drop ceiling slabs) while keeping the
    full cloud for collision elsewhere. ``select_by_index`` leaves ``pcd``
    unchanged.
    """
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return o3d.geometry.PointCloud()
    keep = pts[:, 2] <= float(z_keep_max)
    if not np.any(keep):
        return o3d.geometry.PointCloud()
    return pcd.select_by_index(np.where(keep)[0].tolist())


def z_cut_strip_ceiling(
    pcd: o3d.geometry.PointCloud,
    rel_gap: float = 0.07,
    quantile_cap: Optional[float] = None,
) -> float:
    """Compute a z cutoff to drop a top ``rel_gap`` slab of the height range.

    ``z_hi = z.max() - rel_gap * (z.max() - z.min())`` in aligned coordinates.
    If ``quantile_cap`` is given (e.g. 0.992), use ``min(z_hi, quantile(z, cap))``
    so a thick ceiling cluster is trimmed further.

    Returns ``z_keep_max`` (points with z <= this value are kept).
    """
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        return 0.0
    z = pts[:, 2]
    zmin = float(z.min())
    zmax = float(z.max())
    span = max(zmax - zmin, 1e-9)
    z_hi = zmax - float(rel_gap) * span
    if quantile_cap is not None:
        q = float(np.quantile(z, quantile_cap))
        z_hi = min(z_hi, q)
    return z_hi


# ---------------------------------------------------------------------------
# Ground segmentation
# ---------------------------------------------------------------------------

def ground_alignment_transform(plane_eq: np.ndarray) -> np.ndarray:
    """Return a 4x4 transform that maps a plane ``ax+by+cz+d=0`` to z=0.

    Derivation: let n = (a,b,c)/|(a,b,c)|, d' = d/|(a,b,c)|. For any plane
    point ``n.p + d' = 0`` holds. Pick a rotation R with ``R n = +z``.
    Because R is orthogonal, ``(R p).z = n.p = -d'`` for every plane point,
    so a translation by ``(0, 0, d')`` brings the plane onto ``z = 0``.
    """
    a, b, c, d = [float(x) for x in plane_eq]
    n = np.array([a, b, c], dtype=np.float64)
    n_norm = float(np.linalg.norm(n))
    if n_norm < 1e-9:
        return np.eye(4)
    n = n / n_norm
    d = d / n_norm
    if n[2] < 0:
        n = -n
        d = -d

    z_axis = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z_axis)
    s = float(np.linalg.norm(v))
    c_ = float(np.dot(n, z_axis))
    if s < 1e-9:
        R = np.eye(3) if c_ > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        K = np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])
        R = np.eye(3) + K + K @ K * ((1.0 - c_) / (s * s))

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([0.0, 0.0, d])
    return T


def up_axis_rotation_matrix(up_axis: str = "z") -> np.ndarray:
    """Return 3x3 rotation ``R`` matching :func:`build_environment` ``up_axis`` handling.

    Open3D applies ``p' = R @ (p - c) + c`` with ``c`` the rotation center.
    For ``origin`` rotation, column-vector form is ``p'_col = R @ p_col``.
    Returns ``I`` for ``\"z\"`` / ``\"+z\"`` (already z-up).
    """
    up = up_axis.lower()
    if up == "y":
        return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    if up == "x":
        return np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    if up == "-y":
        return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    if up == "-z":
        return np.diag([1.0, -1.0, -1.0])
    if up in ("z", "+z"):
        return np.eye(3)
    raise ValueError(f"unknown up_axis: {up_axis}")


def segment_ground(
    pcd: o3d.geometry.PointCloud,
    distance_threshold: float = 0.15,
    ransac_n: int = 3,
    num_iterations: int = 1000,
    max_ground_normal_angle_deg: float = 20.0,
    z_quantile: float = 0.30,
) -> Tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray]:
    """Split a cloud into (ground, non_ground, plane_eq) via RANSAC.

    Strategy:
      1. Restrict candidate ground points to the lowest ``z_quantile``
         fraction (by z). This avoids RANSAC locking onto a high-density
         non-ground plane (e.g. a roof, mezzanine, or table) in multi-level
         scenes.
      2. Run RANSAC on those low candidates.
      3. Accept the plane only if its normal is within
         ``max_ground_normal_angle_deg`` of +Z; otherwise fall back to a
         pure low-z slab segmentation.

    Returns: (ground_pcd, non_ground_pcd, plane_eq[a, b, c, d]).
    """
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    z_cut = float(np.quantile(z, z_quantile))
    low_mask = z <= z_cut
    low_idx = np.where(low_mask)[0]
    low = pcd.select_by_index(low_idx.tolist())

    if len(low.points) >= max(ransac_n, 3):
        plane_model, inliers_low = low.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
        )
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal = normal / (np.linalg.norm(normal) + 1e-9)
        cos_angle = abs(float(np.dot(normal, [0.0, 0.0, 1.0])))
        angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

        if angle_deg <= max_ground_normal_angle_deg:
            signed = pts @ np.array([a, b, c]) + d
            inlier_mask = np.abs(signed) <= distance_threshold
            inlier_idx = np.where(inlier_mask)[0].tolist()
            ground = pcd.select_by_index(inlier_idx)
            non_ground = pcd.select_by_index(inlier_idx, invert=True)
            return ground, non_ground, np.asarray(plane_model)

    z_thresh = np.quantile(z, 0.05) + distance_threshold
    mask = z <= z_thresh
    idx = np.where(mask)[0].tolist()
    ground = pcd.select_by_index(idx)
    non_ground = pcd.select_by_index(idx, invert=True)
    plane_eq = np.array([0.0, 0.0, 1.0, -float(np.median(z[mask]))])
    return ground, non_ground, plane_eq


# ---------------------------------------------------------------------------
# Surface (walkable) candidate filtering
# ---------------------------------------------------------------------------

def walkable_surface_mask(
    non_ground: o3d.geometry.PointCloud,
    max_height: float = 18.0,
    max_slope_deg: float = 50.0,
    z_min_above_ground: float = 0.05,
    normal_radii: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """Boolean mask over ``non_ground.points`` for *walkable* surface points.

    A point is walkable if its height is in ``(z_min_above_ground, max_height]``
    and, for **at least one** search radius in ``normal_radii``, its estimated
    normal is within ``max_slope_deg`` of +Z. Multi-radius OR helps thin
    horizontal patches (tabletops, sofa seats) where a single large radius
    mixes in vertical geometry and tilts normals away from vertical.
    """
    pts = np.asarray(non_ground.points)
    n_pts = len(pts)
    if n_pts == 0:
        return np.zeros(0, dtype=bool)

    radii: Tuple[float, ...]
    if normal_radii is not None and len(tuple(normal_radii)) > 0:
        radii = tuple(float(r) for r in normal_radii if float(r) > 0)
        if not radii:
            radii = (0.22, 0.42)
    else:
        radii = (0.22, 0.42)

    combined = np.zeros(n_pts, dtype=bool)
    z_hi = float(max_height)
    z_lo = float(z_min_above_ground)
    max_nn = min(30, max(10, n_pts // 500 + 12))

    for radius in radii:
        tmp = o3d.geometry.PointCloud()
        tmp.points = o3d.utility.Vector3dVector(np.asarray(non_ground.points).copy())
        tmp.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius, max_nn=max_nn
            )
        )
        tmp.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))
        nrm = np.asarray(tmp.normals)
        n_norm = nrm / (np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-9)
        cos_angle = np.abs(n_norm @ np.array([0.0, 0.0, 1.0]))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angles = np.degrees(np.arccos(cos_angle))
        combined |= (
            (angles <= max_slope_deg)
            & (pts[:, 2] <= z_hi)
            & (pts[:, 2] > z_lo)
        )

    return combined



# ---------------------------------------------------------------------------
# Elevation-map based walkable surface detection
# ---------------------------------------------------------------------------
# 方法来源:
#   Fankhauser, P. & Hutter, M. (2018). "A Universal Grid Map Library:
#       Implementation and Use Case for Rough Terrain Navigation." In Robot
#       Operating System (ROS), pp. 99-120.
#   Papadakis, P. (2013). "Terrain traversability analysis methods for
#       unmanned ground vehicles: A survey." Engineering Applications of
#       Artificial Intelligence, 26(4), 1373-1385.
#
# 核心思想:
#   将候选可行走点投影到 XY 平面的规则网格（高程图），在每个网格单元内
#   计算地形统计量（坡度、粗糙度、点密度），然后用阈值判断可行走性。
#   最后通过连通区域面积过滤掉窗台、栏杆等小表面。
#
# 相比旧方法 (prune_walk_mask_by_landing_xy_extent) 的改进:
#   1. 全向量化 numpy 操作，无 Python 逐点循环
#   2. 网格单元大小自动与 voxel_size 对齐，不会出现连通性断裂
#   3. 使用 scipy.ndimage.label 做 8-连通标记，比手写并查集更鲁棒
#   4. 基于面积（m²）过滤，而非 XY 跨度的 AND 条件
#   5. 额外提供坡度和粗糙度两个地形质量指标
# ---------------------------------------------------------------------------


def elevation_map_walkable_mask(
    pts: np.ndarray,
    walk_mask: np.ndarray,
    cell_size: float,
    *,
    max_roughness: float = 0.15,
    max_slope: float = 0.85,
    min_area: float = 0.25,
    min_points_per_cell: int = 1,
) -> np.ndarray:
    """基于高程图的可行走表面过滤。

    在已经通过法向量角度筛选的候选点 (walk_mask=True) 上，进一步用
    地形统计量剔除不适合着陆的区域。

    算法步骤:
      1. 将候选点投影到 cell_size × cell_size 的 XY 网格
      2. 每个网格单元计算:
         - 粗糙度 (roughness): 单元内 z 值的标准差
         - 坡度 (slope): 与相邻单元的高程差 / 水平距离
         - 点密度: 单元内的点数
      3. 标记满足 (roughness <= max_roughness) & (slope <= max_slope)
         & (点数 >= min_points_per_cell) 的单元为可行走。默认 1，
         因为输入通常已经 voxel-downsample，干净水平面在一个高程格
         内经常只有一个代表点。
      4. 用 scipy.ndimage.label 做 8-连通标记，过滤面积 < min_area 的区域
      5. 将网格结果映射回原始点

    Parameters
    ----------
    pts : (N, 3) ndarray
        non_ground 点云的全部坐标。
    walk_mask : (N,) bool ndarray
        由 walkable_surface_mask 产生的初步候选 mask。
    cell_size : float
        高程图网格单元边长（米）。推荐 1.0~1.5 × voxel_size。
    max_roughness : float
        单元内 z 标准差的上限（米）。超过此值认为表面太粗糙。
        参考: Papadakis (2013) 建议 0.05~0.15m 取决于机器人尺寸。
    max_slope : float
        相邻单元间的最大坡度 (dz/dx)。0.85 ≈ tan(40°)。
        参考: 典型四足机器人 tan(35°)≈0.7, 轮式 tan(25°)≈0.47。
    min_area : float
        连通区域的最小面积（m²）。小于此值的区域被视为窗台/栏杆等
        不可着陆表面。
    min_points_per_cell : int
        网格单元内最少需要多少个候选点才被视为有效。默认 1 适合
        已经 voxel-downsample 的点云；真扫噪声很大时可调高。

    Returns
    -------
    (N,) bool ndarray
        过滤后的 walk_mask，只保留通过高程图检验的点。

    References
    ----------
    .. [1] Fankhauser, P. & Hutter, M. (2018). "A Universal Grid Map Library."
    .. [2] Papadakis, P. (2013). "Terrain traversability analysis methods."
    .. [3] Wermelinger, M. et al. (2016). "Navigation planning for legged
           robots in challenging terrain." IROS.
    """
    from scipy.ndimage import label

    pts = np.asarray(pts, dtype=np.float64)
    walk_mask = np.asarray(walk_mask, dtype=bool)
    idx = np.flatnonzero(walk_mask)
    if idx.size == 0:
        return walk_mask

    # ------------------------------------------------------------------
    # Step 1: 将候选点投影到 XY 网格
    # ------------------------------------------------------------------
    sub = pts[idx]  # 只取候选点
    cell = max(float(cell_size), 1e-6)

    # 计算网格坐标（相对于候选点的最小 XY）
    xy_min = sub[:, :2].min(axis=0)
    gx = np.floor((sub[:, 0] - xy_min[0]) / cell).astype(np.int64)
    gy = np.floor((sub[:, 1] - xy_min[1]) / cell).astype(np.int64)

    nx = int(gx.max()) + 1
    ny = int(gy.max()) + 1

    # ------------------------------------------------------------------
    # Step 2: 计算每个网格单元的地形统计量
    # ------------------------------------------------------------------
    # 用 flat index 做向量化聚合，避免 Python 循环
    flat_idx = gx * ny + gy

    # 2a. 高程均值（用于坡度计算）
    z_sum = np.zeros(nx * ny, dtype=np.float64)
    z_count = np.zeros(nx * ny, dtype=np.int64)
    np.add.at(z_sum, flat_idx, sub[:, 2])
    np.add.at(z_count, flat_idx, 1)
    valid_cells = z_count >= min_points_per_cell
    elevation = np.full(nx * ny, np.nan, dtype=np.float64)
    elevation[valid_cells] = z_sum[valid_cells] / z_count[valid_cells]

    # 2b. 粗糙度: 单元内 z 的标准差
    #     roughness = sqrt(E[z²] - E[z]²)
    z_sq_sum = np.zeros(nx * ny, dtype=np.float64)
    np.add.at(z_sq_sum, flat_idx, sub[:, 2] ** 2)
    mean_z = z_sum / np.maximum(z_count, 1)
    mean_z_sq = z_sq_sum / np.maximum(z_count, 1)
    # 方差 = E[z²] - (E[z])²，clip 防止浮点误差产生负值
    variance = np.clip(mean_z_sq - mean_z ** 2, 0.0, None)
    roughness_flat = np.sqrt(variance)

    # 2c. 坡度: 只在相邻有效单元之间计算高程梯度
    #     不使用 Sobel（它会把 NaN 填充值引入边缘），改用直接差分，
    #     并且只在两个相邻单元都有有效高程时才计算坡度。
    #     这避免了障碍物边缘因为旁边是空白区域而被误判为高坡度。
    elev_grid = elevation.reshape(nx, ny)
    valid_grid = valid_cells.reshape(nx, ny)

    # X 方向梯度: (elev[i+1,j] - elev[i-1,j]) / (2*cell)
    # Y 方向梯度: (elev[i,j+1] - elev[i,j-1]) / (2*cell)
    # 只在中心和两侧邻居都有效时才计算
    slope_grid = np.zeros((nx, ny), dtype=np.float64)

    if nx > 2:
        # X 方向: 中心差分
        both_valid_x = valid_grid[:-2, :] & valid_grid[2:, :]
        grad_x = np.zeros((nx, ny), dtype=np.float64)
        grad_x[1:-1, :] = np.where(
            both_valid_x,
            (elev_grid[2:, :] - elev_grid[:-2, :]) / (2.0 * cell),
            0.0,
        )
    else:
        grad_x = np.zeros((nx, ny), dtype=np.float64)

    if ny > 2:
        # Y 方向: 中心差分
        both_valid_y = valid_grid[:, :-2] & valid_grid[:, 2:]
        grad_y = np.zeros((nx, ny), dtype=np.float64)
        grad_y[:, 1:-1] = np.where(
            both_valid_y,
            (elev_grid[:, 2:] - elev_grid[:, :-2]) / (2.0 * cell),
            0.0,
        )
    else:
        grad_y = np.zeros((nx, ny), dtype=np.float64)

    slope_grid = np.sqrt(grad_x ** 2 + grad_y ** 2)
    slope_flat = slope_grid.ravel()

    # ------------------------------------------------------------------
    # Step 3: 标记可行走网格单元
    # ------------------------------------------------------------------
    walkable_cells = (
        valid_cells                              # 有足够的点
        & (roughness_flat <= max_roughness)       # 表面足够平整
        & (slope_flat <= max_slope)               # 坡度在可接受范围内
    )

    # ------------------------------------------------------------------
    # Step 4: 连通区域面积过滤 (8-连通)
    # ------------------------------------------------------------------
    walkable_grid = walkable_cells.reshape(nx, ny)

    # scipy.ndimage.label: 8-连通结构
    structure_8conn = np.ones((3, 3), dtype=int)
    labeled, n_components = label(walkable_grid, structure=structure_8conn)

    if n_components > 0 and min_area > 0:
        # 计算每个连通区域的面积 = 网格单元数 × cell²
        cell_area = cell * cell
        component_sizes = np.bincount(labeled.ravel(), minlength=n_components + 1)
        # label=0 是背景，跳过
        too_small = component_sizes * cell_area < min_area
        too_small[0] = True  # 背景始终排除
        # 将面积不足的区域标记为不可行走
        remove_mask = too_small[labeled]
        walkable_grid[remove_mask] = False

    # ------------------------------------------------------------------
    # Step 5: 将网格结果映射回原始点
    # ------------------------------------------------------------------
    walkable_flat = walkable_grid.ravel()
    # 每个候选点查询其所在网格单元是否可行走
    point_passes = walkable_flat[flat_idx]

    # 构建最终 mask: 原始 walk_mask 中只保留通过高程图检验的点
    result = np.zeros_like(walk_mask, dtype=bool)
    result[idx[point_passes]] = True

    return result


def erode_walkable_surface_mask_xy(
    pts: np.ndarray,
    walk_mask: np.ndarray,
    cell_size: float,
    margin: float,
) -> np.ndarray:
    """Return only the walkable cells far enough from the XY footprint edge."""
    pts = np.asarray(pts, dtype=np.float64)
    walk_mask = np.asarray(walk_mask, dtype=bool)
    idx = np.flatnonzero(walk_mask)
    if idx.size == 0 or margin <= 0.0:
        return walk_mask.copy()

    from scipy.ndimage import distance_transform_edt

    sub = pts[idx]
    cell = max(float(cell_size), 1e-6)
    xy_min = sub[:, :2].min(axis=0)
    gx = np.floor((sub[:, 0] - xy_min[0]) / cell).astype(np.int64)
    gy = np.floor((sub[:, 1] - xy_min[1]) / cell).astype(np.int64)
    nx = int(gx.max()) + 1
    ny = int(gy.max()) + 1

    occ = np.zeros((nx, ny), dtype=bool)
    occ[gx, gy] = True

    # Pad with empty cells so the outer footprint boundary is treated as an
    # edge even when the walkable cells fill the whole local grid.
    padded = np.pad(occ, pad_width=1, mode="constant", constant_values=False)
    dist_cells = distance_transform_edt(padded)[1:-1, 1:-1]
    interior = occ & (dist_cells * cell >= float(margin))
    point_is_interior = interior[gx, gy]

    result = np.zeros_like(walk_mask, dtype=bool)
    result[idx[point_is_interior]] = True
    return result


def local_plane_regression_support(
    pts: np.ndarray,
    walk_mask: np.ndarray,
    *,
    radius: float,
    min_neighbors: int,
    max_rmse: float,
    max_projection: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Confirm walkable points by fitting a local PCA plane around each point.

    Returns ``(support_mask, projected_pts)``. ``support_mask`` is only true for
    original ``walk_mask`` points whose local neighborhood is planar enough.
    ``projected_pts`` contains the same points projected onto their fitted
    local plane, which is useful for landing/clearance samples.
    """
    pts = np.asarray(pts, dtype=np.float64)
    walk_mask = np.asarray(walk_mask, dtype=bool)
    idx = np.flatnonzero(walk_mask)
    projected = pts.copy()
    support = np.zeros_like(walk_mask, dtype=bool)
    if idx.size == 0:
        return support, projected

    from scipy.spatial import cKDTree

    radius = max(float(radius), 1e-9)
    min_neighbors = max(3, int(min_neighbors))
    tree = cKDTree(pts[idx])
    sub = pts[idx]

    for local_i, p in enumerate(sub):
        neigh_local = tree.query_ball_point(p, r=radius)
        if len(neigh_local) < min_neighbors:
            continue
        neigh = sub[np.asarray(neigh_local, dtype=np.int64)]
        centroid = neigh.mean(axis=0)
        centered = neigh - centroid
        cov = centered.T @ centered / max(len(neigh) - 1, 1)
        try:
            eigvals, eigvecs = np.linalg.eigh(cov)
        except np.linalg.LinAlgError:
            continue
        order = np.argsort(eigvals)
        eigvals = eigvals[order]
        normal = eigvecs[:, order[0]]
        n_norm = float(np.linalg.norm(normal))
        if n_norm < 1e-9:
            continue
        normal = normal / n_norm
        residual = centered @ normal
        rmse = float(np.sqrt(np.mean(residual ** 2)))
        point_dist = float((p - centroid) @ normal)
        if rmse > float(max_rmse) or abs(point_dist) > float(max_projection):
            continue
        orig_i = idx[local_i]
        support[orig_i] = True
        projected[orig_i] = p - point_dist * normal

    return support, projected


def filter_walkable_surface(
    non_ground: o3d.geometry.PointCloud,
    max_height: float = 18.0,
    max_slope_deg: float = 50.0,
    voxel_size: float = 0.5,
) -> np.ndarray:
    """Return walkable surface points, downsampled with a voxel grid.

    Wrapper around :func:`walkable_surface_mask`.
    """
    mask = walkable_surface_mask(
        non_ground, max_height, max_slope_deg, z_min_above_ground=0.05
    )
    pts = np.asarray(non_ground.points)
    cand = pts[mask]
    if len(cand) == 0:
        return cand
    cand_pcd = o3d.geometry.PointCloud()
    cand_pcd.points = o3d.utility.Vector3dVector(cand)
    cand_pcd = cand_pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(cand_pcd.points)


def sample_ground_nodes(
    ground: o3d.geometry.PointCloud,
    n_samples: int,
    voxel_size: float = 0.6,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Pick ``n_samples`` ground PRM nodes from a ground point cloud."""
    rng = rng or np.random.default_rng()
    g = ground.voxel_down_sample(voxel_size=voxel_size)
    pts = np.asarray(g.points)
    if len(pts) == 0:
        return pts
    if len(pts) <= n_samples:
        return pts
    idx = rng.choice(len(pts), size=n_samples, replace=False)
    return pts[idx]


def sample_aerial_nodes(
    n_samples: int,
    bounds: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]],
    collision_checker: Optional[CollisionChecker] = None,
    rng: Optional[np.random.Generator] = None,
    max_attempts_factor: int = 5,
) -> np.ndarray:
    """Random uniform aerial samples inside ``bounds``.

    If a ``collision_checker`` is provided we reject samples that already lie
    inside an obstacle (checker called with ``(p, p)``).
    """
    rng = rng or np.random.default_rng()
    (xlo, xhi), (ylo, yhi), (zlo, zhi) = bounds
    zlo = max(zlo, 0.05)

    if collision_checker is None:
        xs = rng.uniform(xlo, xhi, n_samples)
        ys = rng.uniform(ylo, yhi, n_samples)
        zs = rng.uniform(zlo, zhi, n_samples)
        return np.stack([xs, ys, zs], axis=1)

    out = []
    attempts = 0
    max_attempts = max_attempts_factor * n_samples
    while len(out) < n_samples and attempts < max_attempts:
        p = np.array([
            rng.uniform(xlo, xhi),
            rng.uniform(ylo, yhi),
            rng.uniform(zlo, zhi),
        ])
        if not collision_checker(p, p):
            out.append(p)
        attempts += 1
    return np.asarray(out) if out else np.empty((0, 3))


# ---------------------------------------------------------------------------
# Collision checker
# ---------------------------------------------------------------------------

@dataclass
class PointCloudCollisionChecker:
    """KD-Tree based segment-vs-pointcloud collision checker.

    Two layers of checks:

      (1) **3D point-to-cloud check** -- a line segment ``(p1, p2)`` is in
          collision if any point along it (sampled at ``step`` resolution)
          has at least one cloud neighbour within ``r_safe`` in 3D.

      (2) **2.5D heightmap check** (optional but strongly recommended) --
          we project the ``footprint_pcd`` to a discrete xy grid and store
          the max-z in each cell. A point ``(x, y, z)`` is in collision if
          ``z < heightmap(x, y) - r_safe`` (i.e. it sits *below* the
          obstacle's local roof). This catches:

            * ground edges that traverse an obstacle xy footprint at z=0
              (the lower wall has been removed by ``ground_clearance``);
            * aerial samples placed *inside* a closed obstacle volume.

    The class is callable: ``checker(p1, p2) -> bool``.
    """

    obstacle_pcd: o3d.geometry.PointCloud
    r_safe: float = 0.4
    step: float = 0.2
    # 2.5D heightmap barrier
    footprint_pcd: Optional[o3d.geometry.PointCloud] = None
    footprint_voxel: Optional[float] = None  # default: max(step, 0.3)
    footprint_top_clearance: Optional[float] = None  # default: r_safe
    walkable_clear_pcd: Optional[o3d.geometry.PointCloud] = None
    walkable_clear_radius: Optional[float] = None
    # Cells with max-z below this are ignored when building the 2.5D roof map
    # (noise / curb strips near z≈0). Often tied to ``ground_clearance`` in
    # :func:`build_environment` so low clutter does not block the heightmap.
    footprint_min_height: float = 0.4

    def __post_init__(self) -> None:
        self._kd = o3d.geometry.KDTreeFlann(self.obstacle_pcd)
        self._r2 = float(self.r_safe) ** 2
        self._foot_kd = None
        self._foot_heights = None
        self._walk_clear_kd = None
        self._foot_voxel = (
            float(self.footprint_voxel)
            if self.footprint_voxel else max(self.step, 0.3)
        )
        self._foot_step = self._foot_voxel * 0.5
        self._foot_xy_r2 = (self._foot_voxel * 0.75) ** 2
        self._foot_top_clear = (
            float(self.footprint_top_clearance)
            if self.footprint_top_clearance is not None else self.r_safe
        )
        if self.footprint_pcd is not None and len(self.footprint_pcd.points) > 0:
            self._build_heightmap()
        if self.walkable_clear_pcd is not None and len(self.walkable_clear_pcd.points) > 0:
            from scipy.spatial import cKDTree

            pts = np.asarray(self.walkable_clear_pcd.points)
            self._walk_clear_kd = cKDTree(pts)
        self._walk_clear_radius = (
            float(self.walkable_clear_radius)
            if self.walkable_clear_radius is not None
            else max(self.step, 1e-3)
        )

    def _build_heightmap(self) -> None:
        from scipy.spatial import cKDTree
        pts = np.asarray(self.footprint_pcd.points)
        xy = pts[:, :2]
        z = pts[:, 2]
        v = self._foot_voxel
        keys = (xy / v).astype(np.int64)
        flat = keys[:, 0].astype(np.int64) * (1 << 32) + (keys[:, 1].astype(np.int64) + (1 << 31))
        order = np.argsort(flat, kind="stable")
        flat_s = flat[order]
        z_s = z[order]
        keys_s = keys[order]
        boundary = np.concatenate(([0], np.where(np.diff(flat_s) != 0)[0] + 1, [len(flat_s)]))
        cell_max = np.maximum.reduceat(z_s, boundary[:-1])
        cell_keys = keys_s[boundary[:-1]]
        keep = cell_max >= self.footprint_min_height
        cell_keys = cell_keys[keep]
        cell_max = cell_max[keep]
        if len(cell_keys) == 0:
            return
        centers = cell_keys.astype(np.float64) * v + v * 0.5
        self._foot_kd = cKDTree(centers)
        self._foot_heights = cell_max

    def __call__(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        return self.collides(p1, p2)

    def _point_below_roof(self, p: np.ndarray) -> bool:
        if self._foot_kd is None:
            return False
        d2, idx = self._foot_kd.query(p[:2], k=1)
        if d2 * d2 > self._foot_xy_r2:
            return False
        roof_z = float(self._foot_heights[idx])
        return p[2] < roof_z - self._foot_top_clear

    def _point_on_walkable_clearance(self, p: np.ndarray) -> bool:
        if self._walk_clear_kd is None:
            return False
        d, _idx = self._walk_clear_kd.query(p, k=1)
        return float(d) <= self._walk_clear_radius

    def point_collides(self, p: np.ndarray) -> bool:
        p = np.asarray(p, dtype=np.float64)
        k, _idx, dist2 = self._kd.search_knn_vector_3d(p, 1)
        if k > 0 and dist2[0] < self._r2:
            if self._point_on_walkable_clearance(p):
                return False
            return True
        return self._point_below_roof(p)

    def collides(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        p1 = np.asarray(p1, dtype=np.float64)
        p2 = np.asarray(p2, dtype=np.float64)
        seg = p2 - p1
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9:
            return self.point_collides(p1)

        n_steps = max(2, int(np.ceil(seg_len / max(self.step, 1e-3))) + 1)
        ts = np.linspace(0.0, 1.0, n_steps)
        for t in ts:
            if self.point_collides(p1 + seg * t):
                return True
        return False


# ---------------------------------------------------------------------------
# High level convenience
# ---------------------------------------------------------------------------

@dataclass
class PointCloudEnvironment:
    """Bundle everything a PRM needs to plan in a scanned scene."""

    raw_pcd: o3d.geometry.PointCloud
    ground_pcd: o3d.geometry.PointCloud
    non_ground_pcd: o3d.geometry.PointCloud
    surface_pcd: o3d.geometry.PointCloud
    landable_pcd: o3d.geometry.PointCloud
    plane_eq: np.ndarray
    collision_checker: PointCloudCollisionChecker
    bounds: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    ground_to_world: np.ndarray = None  # type: ignore[assignment]
    walkable_count: int = 0
    landable_count: int = 0
    walkable_before_regression_count: int = 0
    regression_count: int = 0

    @property
    def xlim(self) -> Tuple[float, float]:
        return self.bounds[0]

    @property
    def ylim(self) -> Tuple[float, float]:
        return self.bounds[1]

    @property
    def zlim(self) -> Tuple[float, float]:
        return self.bounds[2]


def build_environment(
    pcd_path: str,
    voxel_size: float = 0.2,
    r_safe: float = 0.4,
    collision_step: Optional[float] = None,
    ground_distance_threshold: float = 0.15,
    align_ground_to_z0: bool = True,
    # See docstring: strips a horizontal band near z=0 from *3D* obstacle points.
    ground_clearance: float = 0.3,
    walkable_max_slope_deg: float = 50.0,
    walkable_max_height: float = 100.0,
    surface_lift_distance: Optional[float] = None,
    carve_walkable_surfaces: bool = True,
    walkable_edge_margin: Optional[float] = None,
    obstacle_radius_filter_radius: float = 0.0,
    obstacle_radius_filter_min_neighbors: int = 0,
    surface_regression: bool = False,
    surface_regression_radius: float = 0.5,
    surface_regression_min_neighbors: int = 12,
    surface_regression_max_rmse: float = 0.08,
    surface_regression_max_projection: float = 0.15,
    scale: float = 1.0,
    z_origin: str = "ground",
    up_axis: str = "z",
    strip_ceiling: bool = False,
    ceiling_rel_gap: float = 0.07,
    ceiling_z_max: Optional[float] = None,
    ceiling_quantile: Optional[float] = None,
    walkable_normal_radii: Optional[Sequence[float]] = None,
    z_min_walkable_surface: float = 0.04,
    # --- 高程图方法参数 ---
    elevation_map_max_roughness: float = 0.15,
    elevation_map_max_slope: float = 0.85,
    elevation_map_min_area: float = 0.25,
) -> PointCloudEnvironment:
    """One-shot builder for a complete PRM environment from a PCD file.

    ground_clearance (地面净空带，单位 m)
    -------------------------------------
    在地面已对齐到 ``z=0`` 之后，对 **非地面点云** 构造用于 **3D 碰撞** 的
    ``obstacle_pcd`` 时，会 **丢弃** 满足 ``|z| <= ground_clearance`` 的所有点。

    作用与动机：

    1. **分割与噪声**：RANSAC 地面附近的墙根、踢脚、草地面残点常被分进
       ``non_ground``，却又贴近 ``z=0``。若不删，3D KD 会把「贴地蹭过墙边」的
       路径线段判成碰障。
    2. **几何重叠**：扫描中地面薄片与立墙底部在点云里重叠一层，相当于障碍在
       底部「加宽」，``ground_clearance`` 相当于把这层薄壳从 **3D 障碍点集** 里拿掉。
    3. **与 2.5D 顶面图的分工**：``PointCloudCollisionChecker`` 仍对完整的
       ``non_ground`` 建 heightmap（``footprint_pcd=non_ground``），因此 **从上方
       钻进封闭障碍下方** 仍会被 ``z < roof_z - r_safe`` 挡住；而删掉的是 **低端
       障碍点**，主要缓解「沿地面走 / 低空掠过墙脚」时的误报。

    副作用与取值：

    * **过大**：墙在 ``z ≈ 0`` 附近一整段在 3D 里「掏空」，仅依赖顶盖 heightmap
      时，可能出现与真实几何不完全一致的路径（需结合 ``r_safe`` 与场景高度判断）。
    * **过小 / 零**：干净合成场景可用小值（如 0.05~0.1）保留近地墙面；**吵闹**
      真扫数据常用默认 0.2~0.3。

    另：``footprint_min_height=max(ground_clearance + 0.1, 0.3)`` 让 heightmap 忽略
    过低栅格，避免墙根碎片在 XY 上形成虚假「顶」。

    Steps:
      1. Read + denoise + voxel-downsample.
      2. RANSAC ground-plane segmentation.
      3. (Optional) rotate+translate the cloud so the ground plane becomes
         ``z = 0`` with normal ``+Z``.
      4. (Optional) crop ``z <= z_cut`` on the full cloud, ground, and
         non-ground (``strip_ceiling`` / ``ceiling_z_max`` / quantile).
      5. Detect walkable surface points:
         - 法向量角度筛选 (walkable_surface_mask)
         - 高程图地形质量过滤 (elevation_map_walkable_mask):
           坡度、粗糙度、连通面积
         By default, samples stay directly on the walkable surface. If
         ``surface_lift_distance`` is provided, points are lifted along
         their normal by that amount.
      6. Erode the walkable XY footprint by ``walkable_edge_margin`` before
         using it for surface samples and carving. This keeps obstacle
         inflation on landing-surface edges.
      7. Build the *obstacle* cloud from non-walkable faces only. The full
         walkable plane is removed before 3D inflation; edge safety comes from
         the surrounding non-walkable geometry plus the eroded sampling mask.

    Parameters (elevation map)
    --------------------------
    elevation_map_max_roughness : float
        网格单元内 z 标准差的上限（米）。超过此值认为表面太粗糙。
        参考: Papadakis (2013) 建议 0.05~0.15m。
    elevation_map_max_slope : float
        相邻网格单元间的最大坡度 (dz/dx)。0.85 ≈ tan(40°)。
    elevation_map_min_area : float
        连通区域的最小面积（m²）。小于此值的区域被视为窗台/栏杆。
    """
    # When scaling, load with a *pre-scale* voxel size so that the final
    # post-scale spacing matches the user-requested voxel_size. This keeps
    # all downstream parameters (r_safe, rmax, ground_clearance, ...) in
    # the *scaled* world and stops 'voxel too coarse for original cloud'
    # situations on tiny indoor scans.
    s = float(scale)
    load_voxel = voxel_size / s if s != 1.0 else voxel_size
    pcd = load_pointcloud(pcd_path, voxel_size=load_voxel)
    if s != 1.0:
        pcd.scale(s, center=(0.0, 0.0, 0.0))

    # Convert non-z up-axis conventions (e.g. ICL-NUIM / many CG meshes are
    # y-up) to z-up so that downstream RANSAC + ground alignment do the
    # right thing. Rotation is a permutation only -- no skew / scale.
    R = up_axis_rotation_matrix(up_axis)
    if not np.allclose(R, np.eye(3)):
        pcd.rotate(R, center=(0.0, 0.0, 0.0))
    ground, non_ground, plane_eq = segment_ground(
        pcd, distance_threshold=ground_distance_threshold
    )

    T_world_to_ground = np.eye(4)
    T_ground_to_world = np.eye(4)
    if align_ground_to_z0:
        T_world_to_ground = ground_alignment_transform(plane_eq)
        T_ground_to_world = np.linalg.inv(T_world_to_ground)
        pcd.transform(T_world_to_ground)
        ground.transform(T_world_to_ground)
        non_ground.transform(T_world_to_ground)
        plane_eq = np.array([0.0, 0.0, 1.0, 0.0])

    z_hi = None
    if strip_ceiling:
        z_hi = z_cut_strip_ceiling(
            pcd,
            rel_gap=max(1e-6, float(ceiling_rel_gap)),
            quantile_cap=ceiling_quantile,
        )
    if ceiling_z_max is not None:
        cap = float(ceiling_z_max)
        z_hi = cap if z_hi is None else min(z_hi, cap)
    if z_hi is not None:
        pcd = crop_pcd_below_z(pcd, z_hi)
        ground = crop_pcd_below_z(ground, z_hi)
        non_ground = crop_pcd_below_z(non_ground, z_hi)
        if len(np.asarray(pcd.points)) == 0:
            raise ValueError(
                "Ceiling crop removed all points; loosen ceiling_rel_gap or "
                "raise ceiling_z_max."
            )
        if len(np.asarray(ground.points)) == 0:
            raise ValueError(
                "Ceiling crop removed all ground inliers; raise z_cut "
                "(smaller gap or higher ceiling_z_max)."
            )

    walk_mask = walkable_surface_mask(
        non_ground,
        max_height=walkable_max_height,
        max_slope_deg=walkable_max_slope_deg,
        z_min_above_ground=float(z_min_walkable_surface),
        normal_radii=(
            walkable_normal_radii
            if walkable_normal_radii is not None and len(tuple(walkable_normal_radii)) > 0
            else (
                max(0.12, 1.6 * float(voxel_size)),
                max(0.22, 3.2 * float(voxel_size)),
                min(0.55, 6.0 * float(voxel_size)),
            )
        ),
    )

    # ------------------------------------------------------------------
    # 高程图地形质量过滤 (Fankhauser & Hutter 2018)
    # ------------------------------------------------------------------
    ng_arr = np.asarray(non_ground.points)
    elev_cell = max(float(voxel_size) * 1.2, 0.15)
    walk_mask = elevation_map_walkable_mask(
        ng_arr,
        walk_mask,
        cell_size=elev_cell,
        max_roughness=float(elevation_map_max_roughness),
        max_slope=float(elevation_map_max_slope),
        min_area=float(elevation_map_min_area),
    )
    edge_margin = (
        float(walkable_edge_margin)
        if walkable_edge_margin is not None
        else float(r_safe)
    )
    walkable_before_regression_count = int(np.count_nonzero(walk_mask))
    landable_mask = erode_walkable_surface_mask_xy(
        ng_arr,
        walk_mask,
        cell_size=max(float(voxel_size), 1e-6),
        margin=max(edge_margin, 0.0),
    )
    regression_mask = walk_mask
    regression_projected = ng_arr
    if surface_regression:
        regression_mask, regression_projected = local_plane_regression_support(
            ng_arr,
            walk_mask,
            radius=float(surface_regression_radius),
            min_neighbors=int(surface_regression_min_neighbors),
            max_rmse=float(surface_regression_max_rmse),
            max_projection=float(surface_regression_max_projection),
        )
        walk_mask = walk_mask & regression_mask
        landable_mask = landable_mask & regression_mask

    radii_used = tuple(
        walkable_normal_radii
        if walkable_normal_radii is not None and len(tuple(walkable_normal_radii)) > 0
        else (
            max(0.12, 1.6 * float(voxel_size)),
            max(0.22, 3.2 * float(voxel_size)),
            min(0.55, 6.0 * float(voxel_size)),
        )
    )
    _rfin = float(np.median(np.asarray(radii_used, dtype=np.float64)))
    non_ground.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(0.10, _rfin), max_nn=30
        )
    )
    non_ground.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))

    raw_surf_pts = regression_projected[landable_mask]
    raw_surf_nrm = (
        np.asarray(non_ground.normals)[landable_mask]
        if non_ground.has_normals() else None
    )
    lift = (
        float(surface_lift_distance)
        if surface_lift_distance is not None
        else 0.0
    )
    if len(raw_surf_pts) > 0 and raw_surf_nrm is not None:
        n = raw_surf_nrm / (np.linalg.norm(raw_surf_nrm, axis=1, keepdims=True) + 1e-9)
        flip = n[:, 2] < 0
        n[flip] = -n[flip]
        lifted_pts = raw_surf_pts + n * lift
    elif len(raw_surf_pts) > 0:
        lifted_pts = raw_surf_pts + np.array([0.0, 0.0, lift])
        n = np.tile([0.0, 0.0, 1.0], (len(raw_surf_pts), 1))
    else:
        lifted_pts = raw_surf_pts
        n = raw_surf_nrm
    surface_pcd = o3d.geometry.PointCloud()
    surface_pcd.points = o3d.utility.Vector3dVector(lifted_pts)
    if n is not None and len(n) == len(lifted_pts):
        surface_pcd.normals = o3d.utility.Vector3dVector(n)

    landable_pcd = o3d.geometry.PointCloud()
    landable_pcd.points = o3d.utility.Vector3dVector(raw_surf_pts)
    if raw_surf_nrm is not None and len(raw_surf_nrm) == len(raw_surf_pts):
        landable_pcd.normals = o3d.utility.Vector3dVector(raw_surf_nrm)

    # 3D 碰撞体：默认等于 full non_ground；可按参数剔除低端点 / 可降落平面。
    # 注意这里用 walk_mask（完整可降落平面），不是 landable_mask（采样内核）。
    # 这样可降落平面本身完全不膨胀；边缘安全通过 landable_mask 腐蚀采样，
    # 以及相邻不可降落面（如立面/斜面）的完整膨胀来实现。
    obstacle_pcd = non_ground
    if ground_clearance > 0.0 or carve_walkable_surfaces:
        ng_pts = np.asarray(non_ground.points)
        if len(ng_pts) > 0:
            keep = np.ones(len(ng_pts), dtype=bool)
            if ground_clearance > 0.0:
                # 仅影响 obstacle_pcd（KD 树的 3D 近邻判碰撞），不删 raw non_ground。
                # z 已对齐到地平面为 0：去掉 |z|≤阈值的点 ≈ 去掉贴地的「墙根带」。
                keep &= np.abs(ng_pts[:, 2]) > ground_clearance
            if carve_walkable_surfaces:
                keep &= ~walk_mask
            obstacle_pcd = non_ground.select_by_index(np.where(keep)[0].tolist())

    if (
        obstacle_radius_filter_radius > 0.0
        and obstacle_radius_filter_min_neighbors > 0
        and len(obstacle_pcd.points) > 0
    ):
        obstacle_pcd, _ = obstacle_pcd.remove_radius_outlier(
            nb_points=int(obstacle_radius_filter_min_neighbors),
            radius=float(obstacle_radius_filter_radius),
        )

    coord_z_shift = 0.0
    if z_origin not in ("ground", "min"):
        raise ValueError(f"unknown z_origin: {z_origin!r}")
    if z_origin == "min":
        pts_all = np.asarray(pcd.points)
        if len(pts_all) > 0:
            coord_z_shift = -float(np.min(pts_all[:, 2]))
            if abs(coord_z_shift) > 1e-12:
                shift = np.array([0.0, 0.0, coord_z_shift], dtype=np.float64)
                for pc in (
                    pcd,
                    ground,
                    non_ground,
                    obstacle_pcd,
                    surface_pcd,
                    landable_pcd,
                ):
                    pc.translate(shift)
                T_shift = np.eye(4)
                T_shift[:3, 3] = shift
                T_world_to_ground = T_shift @ T_world_to_ground
                T_ground_to_world = np.linalg.inv(T_world_to_ground)
                plane_eq = np.array([0.0, 0.0, 1.0, -coord_z_shift])

    step = collision_step if collision_step is not None else max(voxel_size, 0.1)
    footprint_min_height = coord_z_shift + max(ground_clearance + 0.1, 0.3)
    checker = PointCloudCollisionChecker(
        obstacle_pcd,
        r_safe=r_safe,
        step=step,
        # heightmap 仍用完整 non_ground：保留「顶面」以检测从下方误入封闭体。
        footprint_pcd=non_ground,
        footprint_voxel=max(voxel_size, 0.3),
        footprint_top_clearance=r_safe,
        walkable_clear_pcd=landable_pcd,
        # This radius only suppresses same-surface false positives around
        # accepted landable samples. Keep it tied to point-cloud resolution,
        # not r_safe; otherwise a large robot radius carves away the inflated
        # vertical sides/edges that should remain obstacles.
        walkable_clear_radius=max(float(voxel_size), 0.1),
        # 与 ground_clearance 联动：过低栅格不参与「顶高」，减少 z≈0 碎点污染。
        footprint_min_height=footprint_min_height,
    )

    pts = np.asarray(pcd.points)
    bounds = (
        (float(pts[:, 0].min()), float(pts[:, 0].max())),
        (float(pts[:, 1].min()), float(pts[:, 1].max())),
        (float(pts[:, 2].min()), float(pts[:, 2].max())),
    )

    return PointCloudEnvironment(
        raw_pcd=pcd,
        ground_pcd=ground,
        non_ground_pcd=obstacle_pcd,
        surface_pcd=surface_pcd,
        landable_pcd=landable_pcd,
        plane_eq=plane_eq,
        collision_checker=checker,
        bounds=bounds,
        ground_to_world=T_ground_to_world,
        walkable_count=int(np.count_nonzero(walk_mask)),
        landable_count=int(np.count_nonzero(landable_mask)),
        walkable_before_regression_count=walkable_before_regression_count,
        regression_count=int(np.count_nonzero(regression_mask)),
    )
