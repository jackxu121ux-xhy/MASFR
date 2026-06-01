from mayavi import mlab
import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev

# 读取参数
from utils.parameters_test import get_parameters
params = get_parameters()

# 创建 Mayavi 图形
mlab.figure(size=(1000, 800), bgcolor=(1.0, 1.0, 1.0))  # 设置背景为白色
mlab.title('3D Path Visualization with WF Interpolation', size=0.5)  # 标题

# 设置坐标轴范围和比例
ax = mlab.axes(
    extent=[params['xlim'][0], params['xlim'][1],
            params['ylim'][0], params['ylim'][1],
            params['zlim'][0], params['zlim'][1]],
    line_width=1.5
)
ax.title_text_properties.color = (0, 0, 0)
ax.title_text_properties.font_family = 'times'

# 坐标轴标签和刻度
mlab.xlabel('X Axis', color=(0, 0, 0))
mlab.ylabel('Y Axis', color=(0, 0, 0))
mlab.zlabel('Z Axis', color=(0, 0, 0))
mlab.draw()

def create_polyhedron(vertices, faces_index):
    vertices = np.array(vertices)
    faces = []
    
    for face_index in faces_index:
        # 每个face的第一个元素应该是该face的顶点数
        face = [len(face_index)] + list(face_index)
        faces.extend(face)

    faces = np.array(faces)
    return faces, vertices

# **障碍物绘制**
def add_expand_obstacles(mlab, params):
    for obstacle_params in params['expand_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        # 将顶点和面转换为 Mayavi 的网格格式
        src = mlab.pipeline.poly_data_source(vertices, faces)
        mlab.pipeline.surface(
            src,
            color=(0.8, 0.8, 0.8),
            opacity=0.3,
            representation='surface'  # 光滑表面
        )

def add_connect_obstacles(mlab, params):
    for obstacle_params in params['connect_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        src = mlab.pipeline.poly_data_source(vertices, faces)
        mlab.pipeline.surface(
            src,
            color=(0.5, 0.5, 1.0),
            opacity=0.5,
            line_width=0.5
        )

def add_real_obstacles(mlab, params):
    for obstacle_params in params['real_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        src = mlab.pipeline.poly_data_source(vertices, faces)
        mlab.pipeline.surface(
            src,
            color=(0.2, 0.2, 0.2),
            opacity=0.6,
            line_width=1.0
        )

# **起点和终点**
def plot_start_and_end(mlab, params):
    start = np.array(params['start'])
    end = np.array(params['end'])
    mlab.points3d(start[0], start[1], start[2], 
                 scale_factor=0.5, 
                 color=(0, 1, 0), 
                 mode='sphere', 
                 opacity=1.0,
                 name='Start')
    mlab.points3d(end[0], end[1], end[2], 
                 scale_factor=0.3, 
                 color=(0, 0, 1), 
                 mode='cone', 
                 opacity=1.0,
                 name='End')

# **路径插值与绘制**
def plot_paths(mlab, df, params):
    current_label = None
    current_segment = []

    COLOR_MAP = {
        "wg": (0, 0, 1),  # 蓝色（RGB）
        "wf": (1, 0, 0)   # 红色（RGB）
    }

    for _, row in df.iterrows():
        coords = row['coordinates']
        label = row['labels']

        if label != current_label:
            if current_segment:
                if current_label == "wf":
                    xs, ys, zs = interpolate_wf_segment(current_segment)
                    mlab.plot3d(xs, ys, zs, 
                               tube_radius=0.05,  
                               color=COLOR_MAP[current_label],
                               opacity=0.9,
                               name=f'Optimized Path ({current_label})')
                else:
                    xs, ys, zs = zip(*current_segment)
                    mlab.plot3d(xs, ys, zs, 
                               tube_radius=0.03, 
                               color=COLOR_MAP[current_label],
                               opacity=0.7,
                               name=f'Global Path ({current_label})')
            current_label = label
            current_segment = [coords]
        else:
            current_segment.append(coords)

    if current_segment:
        if current_label == "wf":
            xs, ys, zs = interpolate_wf_segment(current_segment)
            mlab.plot3d(xs, ys, zs, 
                       tube_radius=0.05, 
                       color=COLOR_MAP[current_label],
                       opacity=0.9,
                       name='Optimized Path')
        else:
            xs, ys, zs = zip(*current_segment)
            mlab.plot3d(xs, ys, zs, 
                       tube_radius=0.03, 
                       color=COLOR_MAP[current_label],
                       opacity=0.7,
                       name='Global Path')

def interpolate_wf_segment(segment):
    path_array = np.array(segment)
    x, y, z = path_array[:, 0], path_array[:, 1], path_array[:, 2]
    
    if len(x) < 4:
        return x, y, z

    spline_degree = min(3, len(x)-1)
    smoothing_factor = 0.0  

    try:
        tck, u = splprep([x, y, z], s=smoothing_factor, k=spline_degree)
        u_new = np.linspace(u.min(), u.max(), num=200)
        xs, ys, zs = splev(u_new, tck)
        return xs, ys, zs
    except Exception as e:
        print(f"插值失败，使用原始路径: {str(e)}")
        return x, y, z

df = pd.read_csv('output.csv')

def parse_point(point_str):
    stripped_str = point_str.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
    coords = [float(c) for c in stripped_str.split()]
    return coords

df['coordinates'] = df['points'].apply(parse_point)

add_expand_obstacles(mlab, params)
add_connect_obstacles(mlab, params)
add_real_obstacles(mlab, params)

plot_start_and_end(mlab, params)

plot_paths(mlab, df, params)

mlab.view(azimuth=-45, elevation=25, distance='auto')  

mlab.text(0.05, 0.95, 'Grand Path', color=(0, 0, 1), width=0.2)
mlab.text(0.05, 0.90, 'Fly Path', color=(1, 0, 0), width=0.2)
mlab.text(0.05, 0.85, 'Start', color=(0, 1, 0), width=0.15)
mlab.text(0.05, 0.80, 'End', color=(0, 0, 1), width=0.15)

mlab.show()