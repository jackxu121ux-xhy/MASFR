# 可行走表面检测方法：学术综述与改进建议

## 当前实现分析

### 现有方法（`utils/env_pointcloud.py`）

当前代码使用的方法：

1. **法向量角度检测** (`walkable_surface_mask`)
   - 估计每个点的法向量
   - 检查法向量与+Z轴的夹角是否 ≤ `max_slope_deg`（默认50°）
   - 使用多个搜索半径（multi-radius OR）来处理薄表面

2. **高度范围过滤**
   - `z_min_above_ground` < z < `max_height`
   - 过滤掉地面和过高的表面

3. **XY平面连通性过滤** (`prune_walk_mask_by_landing_xy_extent`)
   - 基于网格的并查集算法
   - 过滤掉XY方向跨度太小的表面（窗台、栏杆等）

### 问题与局限性

1. **法向量估计不稳定**
   - 对噪声敏感
   - 边缘区域法向量不准确
   - 搜索半径选择影响结果

2. **缺乏几何一致性检查**
   - 没有考虑局部平面拟合质量
   - 没有检查表面粗糙度

3. **连通性算法有缺陷**
   - 参数配置敏感（如前面诊断的问题）
   - 没有考虑高度变化

---

## 学术界方法综述

### 1. 基于地形分析的方法

#### 1.1 Traversability Analysis (可通行性分析)

**经典方法**：
- **论文**: Papadakis, P. (2013). "Terrain traversability analysis methods for unmanned ground vehicles: A survey"
- **方法**: 
  - 局部平面拟合 + 残差分析
  - 坡度、粗糙度、阶梯高度联合评估
  - 使用高程图（elevation map）

**核心指标**：
```python
# 1. 坡度 (Slope)
slope = arctan(sqrt(dz/dx^2 + dz/dy^2))

# 2. 粗糙度 (Roughness) - 点到拟合平面的RMS距离
roughness = sqrt(mean((z_i - z_fitted)^2))

# 3. 阶梯高度 (Step height)
step_height = max(z) - min(z) in local neighborhood

# 4. 曲率 (Curvature)
curvature = eigenvalues of local covariance matrix
```

**引用依据**:
- Papadakis, P. (2013). "Terrain traversability analysis methods for unmanned ground vehicles: A survey". *Engineering Applications of Artificial Intelligence*, 26(4), 1373-1385.

#### 1.2 Elevation Map + Grid-based Analysis

**方法**: 
- 将点云投影到2.5D高程图
- 每个网格单元计算统计特征
- 基于规则或学习的分类器

**优点**:
- 计算效率高
- 易于集成到规划器
- 适合大规模场景

**实现参考**:
- **ANYmal** (ETH Zurich): "Elevation Mapping for Locomotion and Navigation using GPU"
- **论文**: Fankhauser, P., & Hutter, M. (2018). "A Universal Grid Map Library: Implementation and Use Case for Rough Terrain Navigation". *Robot Operating System (ROS)*.

### 2. 基于机器学习的方法

#### 2.1 PointNet-based Classification

**方法**:
- 直接在点云上进行语义分割
- 学习局部几何特征
- 端到端训练

**代表工作**:
- **PointNet++**: Qi, C. R., et al. (2017). "PointNet++: Deep hierarchical feature learning on point sets in a metric space". *NeurIPS*.
- **RandLA-Net**: Hu, Q., et al. (2020). "RandLA-Net: Efficient semantic segmentation of large-scale point clouds". *CVPR*.

**优点**:
- 可以学习复杂的可行走性模式
- 对噪声鲁棒
- 可以处理多种地形类型

**缺点**:
- 需要标注数据
- 计算开销大
- 泛化性能依赖训练数据

#### 2.2 Self-Supervised Learning

**方法**:
- 使用机器人的实际行走数据作为监督信号
- 在线学习可行走性模型

**代表工作**:
- Filitchkin, P., & Byl, K. (2012). "Feature-based terrain classification for LittleDog". *IROS*.
- Wellhausen, L., et al. (2019). "Where should I walk? Predicting terrain properties from images via self-supervised learning". *IEEE Robotics and Automation Letters*.

### 3. 基于几何特征的经典方法

#### 3.1 PCA-based Surface Analysis

**方法**:
- 对局部邻域进行PCA分析
- 使用特征值判断表面类型

**特征值分析**:
```python
# 对局部点集进行PCA
eigenvalues: λ1 >= λ2 >= λ3 (归一化: λ1 + λ2 + λ3 = 1)

# 表面类型判断:
# - 平面: λ3 << λ2 ≈ λ1 (linearity ≈ 0, planarity ≈ 1)
# - 线性: λ2 << λ1, λ3 << λ1 (linearity ≈ 1)
# - 散乱: λ1 ≈ λ2 ≈ λ3 (sphericity ≈ 1)

linearity = (λ1 - λ2) / λ1
planarity = (λ2 - λ3) / λ1
sphericity = λ3 / λ1
```

**可行走性判断**:
- 高planarity + 低sphericity → 可能是可行走表面
- 检查最小特征值对应的特征向量（法向量）与+Z的夹角

**引用依据**:
- Weinmann, M., et al. (2015). "Semantic point cloud interpretation based on optimal neighborhoods, relevant features and efficient classifiers". *ISPRS Journal of Photogrammetry and Remote Sensing*, 105, 286-304.

#### 3.2 RANSAC-based Plane Segmentation

**方法**:
- 使用RANSAC迭代拟合平面
- 评估平面质量（内点数、残差）
- 检查平面方向和大小

**改进版本**:
- **Multi-RANSAC**: 同时拟合多个平面
- **Region Growing**: RANSAC + 区域生长

**引用依据**:
- Schnabel, R., et al. (2007). "Efficient RANSAC for point-cloud shape detection". *Computer Graphics Forum*, 26(2), 214-226.

### 4. 机器人领域的实用方法

#### 4.1 Boston Dynamics / ANYmal 方法

**核心思想**:
- 多层次地形表示
- 实时更新的局部地图
- 基于物理约束的可行走性评估

**关键技术**:
- **Elevation mapping**: 2.5D高程图 + 方差估计
- **Traversability estimation**: 基于坡度、粗糙度、阶梯高度
- **Uncertainty propagation**: 考虑传感器噪声和遮挡

**开源实现**:
- `elevation_mapping` (ROS package by ANYbotics)
- `grid_map` (ROS package by ANYbotics)

**论文**:
- Fankhauser, P., et al. (2018). "Probabilistic terrain mapping for mobile robots with uncertain localization". *IEEE Robotics and Automation Letters*, 3(4), 3019-3026.

#### 4.2 NASA JPL 火星车方法

**GESTALT系统** (Geometric Evaluation of Surface Traversability Applied to Lunar Terrain):
- 多尺度地形分析
- 基于物理模型的可通行性评估
- 考虑车辆几何和动力学约束

**论文**:
- Gorevan, S., et al. (2008). "Rock abrasion tool: Mars Exploration Rover mission". *Journal of Geophysical Research*, 113(E12).
- Goldberg, S. B., et al. (2002). "Stereo vision and rover navigation software for planetary exploration". *IEEE Aerospace Conference*.

---

## 推荐的改进方案

### 方案1：增强的几何特征方法（推荐）

**优点**: 不需要训练数据，计算效率高，可解释性强

#### 实现步骤：

```python
def enhanced_walkable_surface_mask(
    pcd: o3d.geometry.PointCloud,
    max_slope_deg: float = 50.0,
    max_roughness: float = 0.1,  # 米
    min_planarity: float = 0.7,
    search_radius: float = 0.3,
    z_range: Tuple[float, float] = (0.05, 18.0),
) -> np.ndarray:
    """
    基于多个几何特征的可行走表面检测
    
    参考文献:
    - Weinmann et al. (2015) - PCA特征
    - Papadakis (2013) - 地形可通行性分析
    """
    pts = np.asarray(pcd.points)
    n_pts = len(pts)
    
    # 1. 高度过滤
    z_mask = (pts[:, 2] >= z_range[0]) & (pts[:, 2] <= z_range[1])
    
    # 2. 估计法向量（如果没有）
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=search_radius, max_nn=30
            )
        )
        pcd.orient_normals_to_align_with_direction([0, 0, 1])
    
    normals = np.asarray(pcd.normals)
    
    # 3. 坡度检测（法向量与+Z的夹角）
    cos_angle = np.abs(normals @ np.array([0, 0, 1]))
    cos_angle = np.clip(cos_angle, -1, 1)
    slope_angles = np.degrees(np.arccos(cos_angle))
    slope_mask = slope_angles <= max_slope_deg
    
    # 4. PCA特征分析
    tree = o3d.geometry.KDTreeFlann(pcd)
    planarity = np.zeros(n_pts)
    roughness = np.zeros(n_pts)
    
    for i in range(n_pts):
        [k, idx, _] = tree.search_radius_vector_3d(pcd.points[i], search_radius)
        if k < 10:  # 需要足够的邻居点
            continue
        
        neighbors = pts[idx]
        
        # PCA分析
        centroid = neighbors.mean(axis=0)
        centered = neighbors - centroid
        cov = centered.T @ centered / k
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # 归一化特征值（从小到大）
        eigenvalues = np.sort(eigenvalues)
        if eigenvalues.sum() > 1e-9:
            eigenvalues = eigenvalues / eigenvalues.sum()
        
        # 平面性 = (λ2 - λ3) / λ1
        if eigenvalues[2] > 1e-9:
            planarity[i] = (eigenvalues[1] - eigenvalues[0]) / eigenvalues[2]
        
        # 粗糙度 = 点到拟合平面的RMS距离
        # 拟合平面的法向量是最小特征值对应的特征向量
        normal = eigenvectors[:, 0]  # 最小特征值的特征向量
        distances = np.abs((neighbors - centroid) @ normal)
        roughness[i] = np.sqrt(np.mean(distances ** 2))
    
    planarity_mask = planarity >= min_planarity
    roughness_mask = roughness <= max_roughness
    
    # 5. 综合判断
    walkable_mask = z_mask & slope_mask & planarity_mask & roughness_mask
    
    return walkable_mask
```

**关键改进**:
1. **平面性检测**: 使用PCA特征值判断表面是否为平面
2. **粗糙度检测**: 过滤掉粗糙的表面（碎石、灌木等）
3. **多特征融合**: 综合多个几何特征，提高鲁棒性

### 方案2：基于高程图的方法

**优点**: 计算效率高，适合大规模场景，易于集成

#### 实现步骤：

```python
def elevation_map_walkable_detection(
    pcd: o3d.geometry.PointCloud,
    cell_size: float = 0.2,  # 网格单元大小
    max_slope: float = 0.7,  # tan(35°) ≈ 0.7
    max_roughness: float = 0.15,  # 米
    max_step_height: float = 0.3,  # 米
    min_points_per_cell: int = 5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    基于2.5D高程图的可行走表面检测
    
    参考文献:
    - Fankhauser & Hutter (2018) - Grid Map Library
    - ANYbotics elevation_mapping
    
    返回:
    - walkable_mask: 每个点的可行走性标记
    - elevation_map: 高程图（用于可视化和规划）
    """
    pts = np.asarray(pcd.points)
    
    # 1. 构建高程图
    x_min, y_min = pts[:, :2].min(axis=0)
    x_max, y_max = pts[:, :2].max(axis=0)
    
    nx = int(np.ceil((x_max - x_min) / cell_size))
    ny = int(np.ceil((y_max - y_min) / cell_size))
    
    # 网格索引
    gx = ((pts[:, 0] - x_min) / cell_size).astype(int)
    gy = ((pts[:, 1] - y_min) / cell_size).astype(int)
    gx = np.clip(gx, 0, nx - 1)
    gy = np.clip(gy, 0, ny - 1)
    
    # 每个网格单元的统计信息
    elevation_map = np.full((nx, ny), np.nan)
    variance_map = np.full((nx, ny), np.nan)
    slope_map = np.full((nx, ny), np.nan)
    roughness_map = np.full((nx, ny), np.nan)
    
    # 计算每个网格单元的统计量
    from collections import defaultdict
    cells = defaultdict(list)
    for i, (ix, iy) in enumerate(zip(gx, gy)):
        cells[(ix, iy)].append(i)
    
    for (ix, iy), indices in cells.items():
        if len(indices) < min_points_per_cell:
            continue
        
        cell_pts = pts[indices]
        z_values = cell_pts[:, 2]
        
        # 高程（均值或中位数）
        elevation_map[ix, iy] = np.median(z_values)
        
        # 方差
        variance_map[ix, iy] = np.var(z_values)
        
        # 粗糙度（标准差）
        roughness_map[ix, iy] = np.std(z_values)
    
    # 2. 计算坡度（使用Sobel算子）
    from scipy.ndimage import sobel
    
    # 填充NaN值（用于梯度计算）
    elevation_filled = elevation_map.copy()
    mask = ~np.isnan(elevation_filled)
    if mask.sum() > 0:
        from scipy.ndimage import distance_transform_edt
        # 简单的最近邻插值
        ind = distance_transform_edt(~mask, return_distances=False, return_indices=True)
        elevation_filled = elevation_filled[tuple(ind)]
    
    # 计算梯度
    grad_x = sobel(elevation_filled, axis=0) / cell_size
    grad_y = sobel(elevation_filled, axis=1) / cell_size
    slope_map = np.sqrt(grad_x**2 + grad_y**2)
    
    # 3. 可行走性判断
    walkable_grid = (
        (slope_map <= max_slope) &
        (roughness_map <= max_roughness) &
        (~np.isnan(elevation_map))
    )
    
    # 4. 将网格结果映射回点
    walkable_mask = np.zeros(len(pts), dtype=bool)
    for i, (ix, iy) in enumerate(zip(gx, gy)):
        if walkable_grid[ix, iy]:
            walkable_mask[i] = True
    
    return walkable_mask, {
        'elevation': elevation_map,
        'slope': slope_map,
        'roughness': roughness_map,
        'walkable': walkable_grid,
        'cell_size': cell_size,
        'origin': (x_min, y_min),
    }
```

### 方案3：混合方法（最佳实践）

结合几何特征和高程图的优点：

```python
def hybrid_walkable_detection(
    pcd: o3d.geometry.PointCloud,
    # 几何特征参数
    max_slope_deg: float = 50.0,
    min_planarity: float = 0.6,
    max_roughness_local: float = 0.1,
    search_radius: float = 0.3,
    # 高程图参数
    cell_size: float = 0.2,
    max_slope_global: float = 0.7,
    max_roughness_global: float = 0.15,
    # 连通性参数
    min_area: float = 0.5,  # 平方米
) -> np.ndarray:
    """
    混合方法：局部几何特征 + 全局高程图 + 连通性分析
    
    优点：
    - 局部特征捕捉细节（边缘、小障碍）
    - 全局特征提供上下文（坡度、连通性）
    - 多层次验证，提高鲁棒性
    """
    # 1. 局部几何特征检测
    local_mask = enhanced_walkable_surface_mask(
        pcd, max_slope_deg, max_roughness_local,
        min_planarity, search_radius
    )
    
    # 2. 全局高程图检测
    global_mask, elevation_info = elevation_map_walkable_detection(
        pcd, cell_size, max_slope_global, max_roughness_global
    )
    
    # 3. 取交集（两种方法都认为可行走）
    combined_mask = local_mask & global_mask
    
    # 4. 连通性过滤（改进版）
    if min_area > 0:
        combined_mask = filter_by_connected_area(
            pcd, combined_mask, min_area, cell_size
        )
    
    return combined_mask


def filter_by_connected_area(
    pcd: o3d.geometry.PointCloud,
    mask: np.ndarray,
    min_area: float,
    cell_size: float,
) -> np.ndarray:
    """
    改进的连通性过滤：基于实际面积而不是XY跨度
    
    修复原始 prune_walk_mask_by_landing_xy_extent 的问题：
    - 使用合理的cell_size（不会太小）
    - 基于面积判断（而不是XY跨度的AND）
    - 考虑3D连通性
    """
    pts = np.asarray(pcd.points)
    walkable_pts = pts[mask]
    
    if len(walkable_pts) == 0:
        return mask
    
    # 投影到XY平面并网格化
    xy = walkable_pts[:, :2]
    xy_min = xy.min(axis=0)
    
    gx = ((xy[:, 0] - xy_min[0]) / cell_size).astype(int)
    gy = ((xy[:, 1] - xy_min[1]) / cell_size).astype(int)
    
    # 构建网格占用图
    from collections import defaultdict
    grid_to_points = defaultdict(list)
    walkable_indices = np.where(mask)[0]
    
    for i, (ix, iy) in enumerate(zip(gx, gy)):
        grid_to_points[(ix, iy)].append(walkable_indices[i])
    
    # 并查集找连通区域
    parent = {cell: cell for cell in grid_to_points}
    
    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]
    
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
    
    # 8-连通（而不是4-连通）
    for (cx, cy) in grid_to_points:
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (cx + dx, cy + dy)
                if neighbor in grid_to_points:
                    union((cx, cy), neighbor)
    
    # 计算每个连通区域的面积
    components = defaultdict(list)
    for cell in grid_to_points:
        components[find(cell)].append(cell)
    
    # 过滤小区域
    surviving_mask = np.zeros_like(mask)
    for cells in components.values():
        area = len(cells) * (cell_size ** 2)
        if area >= min_area:
            for cell in cells:
                for idx in grid_to_points[cell]:
                    surviving_mask[idx] = True
    
    return surviving_mask
```

---

## 实施建议

### 短期（立即可用）

1. **修复当前实现的参数问题**（已完成）
   - 设置 `walkable_min_landing_xy = 0` 或合理值
   - 显式设置 `walkable_cluster_cell_xy >= voxel_size`

2. **添加粗糙度检测**
   ```python
   # 在 walkable_surface_mask 中添加
   roughness = compute_local_roughness(non_ground, search_radius)
   roughness_mask = roughness <= max_roughness
   combined_mask &= roughness_mask
   ```

### 中期（1-2周）

3. **实现增强的几何特征方法**（方案1）
   - 添加PCA平面性检测
   - 添加粗糙度检测
   - 保持向后兼容

4. **改进连通性过滤**
   - 使用8-连通而不是4-连通
   - 基于面积而不是XY跨度
   - 自动选择合理的cell_size

### 长期（1-2月）

5. **实现高程图方法**（方案2）
   - 构建2.5D高程图
   - 计算坡度、粗糙度、阶梯高度
   - 提供可视化和调试工具

6. **集成混合方法**（方案3）
   - 结合局部和全局特征
   - 提供多种预设配置
   - 添加自动参数调优

---

## 参数调优指南

### 关键参数及其影响

| 参数 | 典型值 | 影响 | 调优建议 |
|------|--------|------|----------|
| `max_slope_deg` | 30-50° | 过小：漏检斜坡；过大：误检墙面 | 根据机器人爬坡能力设置 |
| `max_roughness` | 0.05-0.15m | 过小：漏检粗糙地面；过大：误检障碍 | 根据点云密度和噪声水平调整 |
| `min_planarity` | 0.6-0.8 | 过高：漏检；过低：误检 | 0.7是一个好的起点 |
| `search_radius` | 1-3× voxel_size | 过小：噪声敏感；过大：丢失细节 | 通常设为2× voxel_size |
| `cell_size` | 0.5-1.5× voxel_size | 影响高程图分辨率 | 与voxel_size保持一致 |
| `min_area` | 0.3-1.0 m² | 过小：保留小表面；过大：漏检 | 根据机器人尺寸设置 |

### 不同场景的推荐配置

#### 室内场景（办公室、走廊）
```yaml
max_slope_deg: 30.0
max_roughness: 0.08
min_planarity: 0.75
min_area: 0.3
```

#### 室外场景（草地、碎石路）
```yaml
max_slope_deg: 45.0
max_roughness: 0.15
min_planarity: 0.60
min_area: 0.5
```

#### 工业场景（仓库、工厂）
```yaml
max_slope_deg: 35.0
max_roughness: 0.10
min_planarity: 0.70
min_area: 0.4
```

---

## 参考文献

### 综述论文
1. Papadakis, P. (2013). "Terrain traversability analysis methods for unmanned ground vehicles: A survey". *Engineering Applications of Artificial Intelligence*, 26(4), 1373-1385.

2. Borges, P., et al. (2015). "A survey on terrain traversability analysis for autonomous ground vehicles: Machine learning, geometric, and visual methods". *Unmanned Systems*, 3(02), 125-146.

### 几何方法
3. Weinmann, M., et al. (2015). "Semantic point cloud interpretation based on optimal neighborhoods, relevant features and efficient classifiers". *ISPRS Journal of Photogrammetry and Remote Sensing*, 105, 286-304.

4. Schnabel, R., et al. (2007). "Efficient RANSAC for point-cloud shape detection". *Computer Graphics Forum*, 26(2), 214-226.

### 高程图方法
5. Fankhauser, P., & Hutter, M. (2018). "A Universal Grid Map Library: Implementation and Use Case for Rough Terrain Navigation". *Robot Operating System (ROS)*, 99-120.

6. Fankhauser, P., et al. (2018). "Probabilistic terrain mapping for mobile robots with uncertain localization". *IEEE Robotics and Automation Letters*, 3(4), 3019-3026.

### 机器学习方法
7. Qi, C. R., et al. (2017). "PointNet++: Deep hierarchical feature learning on point sets in a metric space". *NeurIPS*.

8. Wellhausen, L., et al. (2019). "Where should I walk? Predicting terrain properties from images via self-supervised learning". *IEEE Robotics and Automation Letters*, 4(2), 1509-1516.

### 开源实现
- **ANYbotics elevation_mapping**: https://github.com/ANYbotics/elevation_mapping
- **grid_map**: https://github.com/ANYbotics/grid_map
- **Open3D**: http://www.open3d.org/
- **PCL (Point Cloud Library)**: https://pointclouds.org/

---

## 总结

当前实现使用的方法（法向量角度 + 高度过滤 + 连通性）是一个合理的起点，但存在以下问题：

1. **参数配置错误**导致连通性算法失效（已修复）
2. **缺乏粗糙度和平面性检测**，容易误检
3. **连通性算法设计不够鲁棒**

**推荐的改进路径**：
1. 短期：修复参数 + 添加粗糙度检测
2. 中期：实现增强的几何特征方法（PCA + 多特征融合）
3. 长期：实现混合方法（局部特征 + 高程图 + 连通性）

这些方法都有学术文献支持，并在实际机器人系统中得到验证（ANYmal、Boston Dynamics等）。