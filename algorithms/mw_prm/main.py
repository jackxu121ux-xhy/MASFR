from utils.parameters_test import get_parameters
from utils.plotting import setup_plot, add_expand_obstacles, plot_start_and_end, add_real_obstacles, add_connect_obstacles 
from utils.graph import generate_nodes, connect_nearby_nodes
from utils.path_planning import astar, edge_is_pure_walk, smooth_path
import matplotlib.pyplot as plt
import csv

def main():
    # 获取参数
    params = get_parameters()

    # 设置图形
    ax, fig = setup_plot(params)

    # 添加障碍物
    expand_obstacles, obstacles_vertices = add_expand_obstacles(ax, params)
    real_obstacles, real_obstacles_vertices = add_real_obstacles(ax, params)
    connect_obstacles, real_obstacles_vertices = add_connect_obstacles(ax, params)

    # 绘制起点和终点
    plot_start_and_end(ax, params)

    # 生成节点
    samples, start_idx, end_idx, obstacles_surface_points = generate_nodes(
        params, connect_obstacles, total_points=500, area_threshold=0, max_height=18, threshold_angle=85)
    
    # # 可视化所有节点
    # ax.scatter(obstacles_surface_points[:, 0], obstacles_surface_points[:, 1], obstacles_surface_points[:, 2], c='gray', s=10, label='Nodes')

    # 连接邻近节点
    edges, weights = connect_nearby_nodes(samples, params, expand_obstacles, obstacles_surface_points)

    #  # 可视化所有边
    # for node1, connected_nodes in edges.items():
    #     for node2 in connected_nodes:
    #         edge = frozenset([node1, node2])
    #         weight = weights[edge]
    #         if weight == params['wg']:
    #             ax.plot(
    #                 [node1[0], node2[0]],
    #                 [node1[1], node2[1]],
    #                 [node1[2], node2[2]],
    #                 color='green',
    #                 linestyle='-',
    #                 linewidth=0.5,
    #             )


    # 找路径
    path = astar(tuple(samples[start_idx]), tuple(samples[end_idx]), edges, weights, params)
    print(path)

    # 分别绘制两种类型的边
    data_rows = []
    wg_path = []
    wf_path = []
    for i in range(len(path) - 1):
        start_point = path[i]
        end_point = path[i + 1]
        edge = frozenset([start_point, end_point])
        weight = weights[edge]
        edge_1 = None
        if end_point != path[-1]:
            edge_1 = frozenset([path[i + 1], path[i + 2]])

        if edge_is_pure_walk(weight):
            wg_path.append(start_point)
            if edge_1 is None or not edge_is_pure_walk(weights[edge_1]):
                wg_path.append(end_point)
                wg_smoothed_path = smooth_path(wg_path, type="wg")
                for i in range(len(wg_smoothed_path) - 1):
                    start_point = wg_smoothed_path[i]
                    end_point = wg_smoothed_path[i + 1]
                    ax.plot(
                        [start_point[0], end_point[0]],
                        [start_point[1], end_point[1]],
                        [start_point[2], end_point[2]],
                        color='blue',
                        linestyle='-',
                        linewidth=2.0,
                    )
                for point in wg_smoothed_path:
                    data_rows.append([str(point), "wg"])
            
                wg_path = []
        else:
            wf_path.append(start_point)
            if edge_1 is None or edge_is_pure_walk(weights[edge_1]):
                wf_path.append(end_point)
                wf_smoothed_path = smooth_path(wf_path, type='wf')
                for i in range(len(wf_smoothed_path) - 1):
                    start_point = wf_smoothed_path[i]
                    end_point = wf_smoothed_path[i + 1]
                    ax.plot(
                        [start_point[0], end_point[0]],
                        [start_point[1], end_point[1]],
                        [start_point[2], end_point[2]],
                        color='red',
                        linestyle='-',
                        linewidth=2.0,
                    )
                for point in wf_smoothed_path:
                    data_rows.append([point, "wf"])
                wf_path = []

    # 显示图例
    ax.legend()
    plt.show()

    with open('output.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['points', 'labels'])  # 写入列名
        writer.writerows(data_rows)

if __name__ == "__main__":
    main()
