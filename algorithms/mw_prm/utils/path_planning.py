import numpy as np
from scipy.interpolate import splprep, splev

# Edge weight layout stored by ``utils.graph.connect_nearby_nodes``
# ----------------------------------------------------------------------
# · ("wg", c)               walking / ground-plane style edge, cost = d * c
# · ("wf", c)               pure aerial segment,               cost = d * c
# · ("sw", c, penalty)      mode switch (mixed z etc.), cost = d * c + penalty
#
# Older code used plain floats plus list [wf, e]; floats are ambiguous when
# wf == wg — never use scalar floats for modality routing.


def mode_switch_penalty(params: dict) -> float:
    return float(params["wg"]) * float(params["e_factor"])


def edge_traversal_cost(distance: float, weight) -> float:
    """Traversal cost along one graph edge."""
    dist = float(distance)
    if isinstance(weight, tuple) and weight:
        tag = weight[0]
        if tag == "wg":
            return dist * float(weight[1])
        if tag == "wf":
            return dist * float(weight[1])
        if tag == "sw":
            return dist * float(weight[1]) + float(weight[2])
    # legacy multimodal tuples from older checkpoints
    if isinstance(weight, list) and len(weight) == 2:
        return dist * float(weight[0]) + float(weight[1])
    raise TypeError(
        f"Unsupported edge weight {weight!r}; "
        "expected ('wg',c), ('wf',c), ('sw',c,p), or [wf,e]."
    )


def edge_is_pure_walk(weight) -> bool:
    """True iff the PRM annotates this edge as walking (turquoise tubes / wg PRM viz)."""
    return isinstance(weight, tuple) and len(weight) >= 1 and weight[0] == "wg"


def path_segment_kind(weight) -> str:
    """Return ``'wg'`` or ``'wf'`` for path splitting / smoothing (wf includes switches)."""
    if isinstance(weight, tuple) and weight:
        tag = weight[0]
        if tag == "wg":
            return "wg"
        if tag in ("wf", "sw"):
            return "wf"
    if isinstance(weight, list) and len(weight) == 2:
        return "wf"
    raise TypeError(f"Unsupported edge weight for path modality: {weight!r}")


# MM_A* 算法
def astar(start, goal, edges, weights, params):
    wg = float(params["wg"])
    wf = float(params["wf"])
    e_pen = mode_switch_penalty(params)

    def heuristic_MM(point1, point2):
        if point1 == point2:
            return 0
        edge = frozenset([point1, point2])
        w = weights[edge]
        d = heuristic(point1, point2)
        return edge_traversal_cost(d, w)

    def heuristic_H(point, goal):
        if point == goal:
            return 0
        if point[2] == 0 and goal[2] == 0:
            h_cost = wg * (
                np.sqrt((point[0] - goal[0]) ** 2 + (point[1] - goal[1]) ** 2)
            )
        elif point[2] == 0 and goal[2] != 0:
            h_cost = (
                wg * np.sqrt((point[0] - goal[0]) ** 2 + (point[1] - goal[1]) ** 2)
                + wf * abs(point[2] - goal[2])
                + e_pen
            )
        elif point[2] != 0 and goal[2] == 0:
            h_cost = (
                wg * np.sqrt((point[0] - goal[0]) ** 2 + (point[1] - goal[1]) ** 2)
                + wf * abs(point[2] - goal[2])
                + e_pen
            )
        elif point[2] != 0 and goal[2] != 0:
            h_cost_1 = (
                wg * np.sqrt((point[0] - goal[0]) ** 2 + (point[1] - goal[1]) ** 2)
                + wf * abs(point[2])
                + wf * abs(goal[2])
                + 2 * e_pen
            )
            h_cost_2 = wf * heuristic(point, goal)
            h_cost = min(h_cost_1, h_cost_2)
        else:
            h_cost = 0.0
        return float(h_cost)

    open_set = set()
    closed_set = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic_H(start, goal)}

    open_set.add(start)

    while open_set:
        current = min(open_set, key=lambda o: (f_score[o], o[0], o[1], o[2]))

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in edges[current]:
            tentative_g_score = g_score[current] + heuristic_MM(current, neighbor)

            if neighbor in closed_set:
                continue

            if neighbor not in open_set:
                open_set.add(neighbor)

            if tentative_g_score >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score + heuristic_H(neighbor, goal)

    return None


# 计算节点间距离
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def smooth_path(path, type, smoothing_factor=5, spline_degree=3, lift_epsilon=1.0):
    """使用B样条插值平滑给定路径，并确保起点和终点处的切线沿z轴方向且更加明显。
    
    参数:
        path (list of tuples): 输入路径点，每个点为(x, y, z)格式。
        type (str): 类型标识符，"wf" 表示添加沿z轴方向的切线条件。
        smoothing_factor (float): 平滑因子，值越小曲线越接近原始数据点。
        spline_degree (int): 样条次数，默认为3（三阶）。
        lift_epsilon (float): 起飞 / 降落附近沿 z 轴的虚拟控制点增量 (m)。
            仅在 ``type == "wf"`` 时生效。值越大起降越"垂直"，值越小越"斜飞"。
    
    返回:
        ndarray: 平滑后的路径点。
    """
    
    path_array = np.array(path)
    x, y, z = path_array[:, 0], path_array[:, 1], path_array[:, 2]

    # 更新路径长度
    length = len(x)
    
    if type == "wf":
        # 添加额外的控制点以确保起点和终点处的切线沿z轴方向
        start_point = path[0]
        end_point = path[-1]
        
        # 沿 z 轴方向的虚拟控制点增量，决定起降的"垂直程度"
        epsilon = float(lift_epsilon)
        num_control = int(len(z)/3)

        for j in range(len(z)-2):
            z[j+1] = z[j+1] + num_control * epsilon
           
            

        for i in range(num_control): 
            # 在起点附近添加多个控制点，使切线沿正z轴方向
            x = np.insert(x, i+1, start_point[0])
            y = np.insert(y, i+1, start_point[1])
            z = np.insert(z, i+1, start_point[2] + (i+1)*epsilon)
        
            # 在终点附近添加多个控制点，使切线沿负z轴方向（假设你想要相反方向）
            x = np.insert(x, -(i+1), end_point[0])
            y = np.insert(y, -(i+1), end_point[1])
            z = np.insert(z, -(i+1), end_point[2] + (i+1) * epsilon)
        # 给起始点和终点附近的控制点赋予更高的权重
        weights = np.ones(len(x))
        weights[:num_control] = 5  # 起始点附近权重设为5
        weights[-(num_control+1):] = 5  # 结束点附近权重设为5
        smoothed_path = np.column_stack((x, y, z))
        return smoothed_path
        
    else:
        weights = None

    
    if length <= spline_degree:
        spline_degree = max(1, length - 1)  # 确保样条次数至少为1
    
    tck, u = splprep([x, y, z], s=smoothing_factor, per=False, k=spline_degree, w=weights)
    
    # 生成新的参数值以获得更高分辨率的路径
    u_new = np.linspace(u.min(), u.max(), num=length * 10)
    x_new, y_new, z_new = splev(u_new, tck, der=0)
    
    smoothed_path = np.column_stack((x_new, y_new, z_new))
    return smoothed_path
