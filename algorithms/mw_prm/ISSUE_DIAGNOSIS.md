# 问题诊断：所有表面节点被过滤掉

## 问题描述

在运行 `main_pcd.py` 时，所有的可行走表面节点（surface nodes）都被过滤掉了，导致无法生成表面采样点。

## 根本原因

问题出在 `utils/env_pointcloud.py` 的 `build_environment` 函数（第724-755行）中调用的 `prune_walk_mask_by_landing_xy_extent` 函数。

### 详细分析

1. **函数目的**：`prune_walk_mask_by_landing_xy_extent` 的目的是过滤掉XY方向跨度太小的可行走表面（例如窗台、栏杆等）。

2. **算法逻辑**：
   - 将可行走点投影到 `cell_xy × cell_xy` 的网格上
   - 使用并查集找到4-连通的网格单元组
   - 对每个连通区域，计算其在X和Y方向上的跨度
   - **只保留那些 `dx_span >= min_span_xy AND dy_span >= min_span_xy` 的区域**

3. **并查集连通性**：
   ```python
   for cx, cy in buckets:
       c = (cx, cy)
       if (cx + 1, cy) in buckets:  # 只连接右边相邻的网格
           union(c, (cx + 1, cy))
       if (cx, cy + 1) in buckets:  # 只连接下边相邻的网格
           union(c, (cx, cy + 1))
   ```
   
   **关键点**：并查集只连接网格坐标差为1的相邻网格单元。

4. **问题所在**：
   
   当前配置（`yml/pcd/synth_parameters_obstacles.yml`）：
   ```yaml
   voxel: 0.3                      # 点间距约为0.3米
   walkable_min_landing_xy: 0.01   # min_span_xy = 0.01米
   walkable_cluster_cell_xy: null  # 未设置，会自动计算
   ```

   自动计算的 `cell_xy`：
   ```python
   wl = 0.01
   cell_land = max(1e-4, min(voxel_size * 0.4, wl / 14.0))
             = max(0.0001, min(0.12, 0.000714))
             = 0.000714 米
   ```

   **问题**：
   - 点间距 ≈ 0.3 米（voxel_size）
   - cell_xy = 0.000714 米
   - 相邻两点的网格坐标差 = floor(0.3 / 0.000714) ≈ 420
   
   这意味着相邻的两个点之间有419个空网格单元！并查集无法连接它们，因为它只连接网格坐标差为1的单元。

   **结果**：每个点都成为一个孤立的连通区域，其 `dx_span = dy_span = 0`，因此所有点都被过滤掉了。

## 解决方案

### 方案1：禁用过滤器（推荐用于调试）

修改 `yml/pcd/synth_parameters_obstacles.yml`：
```yaml
walkable_min_landing_xy: 0.0  # 禁用过滤器
```

**优点**：简单直接，保留所有可行走表面
**缺点**：无法过滤掉小的表面（如窗台）

### 方案2：修复参数配置

修改 `yml/pcd/synth_parameters_obstacles.yml`：
```yaml
walkable_min_landing_xy: 0.5        # 50厘米，合理的最小表面尺寸
walkable_cluster_cell_xy: 0.35      # 略大于voxel_size，确保相邻点被连接
```

**说明**：
- `cell_xy` 应该 >= `voxel_size`，以确保相邻点落在相同或相邻的网格单元中
- `min_span_xy` 应该是几十厘米到1米，以过滤掉真正的小表面

### 方案3：修改代码逻辑（需要修改源代码）

修改 `utils/env_pointcloud.py` 第745-749行的自动计算逻辑：

```python
# 原代码
wl = float(walkable_min_landing_xy)
cell_land = max(
    1e-4,
    min(float(voxel_size) * 0.4, wl / 14.0),
)

# 修改为
wl = float(walkable_min_landing_xy)
cell_land = max(
    float(voxel_size) * 0.8,  # 确保 >= voxel_size
    min(float(voxel_size) * 1.2, wl / 10.0),
)
```

**优点**：自动计算出合理的 `cell_xy` 值
**缺点**：需要修改源代码

## 参数设计原则

1. **cell_xy 的选择**：
   - 应该 >= 点间距（通常是 voxel_size）
   - 太小：破坏连通性，导致所有点被过滤
   - 太大：可能将不相邻的点错误地连接起来
   - 推荐值：`voxel_size` 到 `1.5 * voxel_size`

2. **min_span_xy 的选择**：
   - 取决于你想过滤掉多小的表面
   - 窗台、栏杆：通常 < 0.5 米
   - 桌面、平台：通常 > 0.5 米
   - 推荐值：0.5 - 1.0 米

3. **两者的关系**：
   - `cell_xy` 控制连通性判断的粒度
   - `min_span_xy` 控制过滤的阈值
   - 它们应该独立设置，不应该让 `min_span_xy` 影响 `cell_xy` 的计算

## 测试验证

修改配置后，运行：
```bash
python main_pcd.py --yml yml/pcd/synth_parameters_obstacles.yml
```

检查输出中的 `surface points` 数量是否 > 0。

## 相关代码位置

- 问题函数：[utils/env_pointcloud.py:313-392](utils/env_pointcloud.py#L313-L392) `prune_walk_mask_by_landing_xy_extent`
- 调用位置：[utils/env_pointcloud.py:740-755](utils/env_pointcloud.py#L740-L755) `build_environment`
- 参数计算：[utils/env_pointcloud.py:745-749](utils/env_pointcloud.py#L745-L749)
- 配置文件：[yml/pcd/synth_parameters_obstacles.yml:30](yml/pcd/synth_parameters_obstacles.yml#L30)
