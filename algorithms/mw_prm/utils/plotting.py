import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def setup_plot(params, tick_interval=25):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(params['xlim'])
    ax.set_ylim(params['ylim'])
    ax.set_zlim(params['zlim'])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xticks([params['xlim'][0] + i * tick_interval for i in range(int((params['xlim'][1] - params['xlim'][0]) // tick_interval) + 1)])
    ax.set_yticks([params['ylim'][0] + i * tick_interval for i in range(int((params['ylim'][1] - params['ylim'][0]) // tick_interval) + 1)])
    ax.set_zticks([params['zlim'][0] + i * tick_interval for i in range(int((params['zlim'][1] - params['zlim'][0]) // tick_interval) + 1)])
    ax.set_box_aspect([abs(lim[1] - lim[0]) for lim in [params['xlim'], params['ylim'], params['zlim']]])
    return ax, fig

def add_expand_obstacles(ax, params):
    obstacles = []
    obstacles_vertices = []
    for obstacle_params in params['expand_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        obstacles.append(faces)
        obstacles_vertices.append(vertices)
        # ax.add_collection3d(Poly3DCollection(faces, facecolors='red', linewidths=0.5, edgecolors=[0.1, 0.1, 0.1], alpha=.1))
    return obstacles, obstacles_vertices

def add_connect_obstacles(ax, params):
    real_obstacles = []
    real_obstacles_vertices = []
    for obstacle_params in params['connect_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        real_obstacles.append(faces)
        real_obstacles_vertices.append(vertices)
        ax.add_collection3d(Poly3DCollection(faces, facecolors='blue', linewidths=0.5, edgecolors=[0.1, 0.1, 0.1], alpha=.1))
    return real_obstacles, real_obstacles_vertices

def add_real_obstacles(ax, params):
    real_obstacles = []
    real_obstacles_vertices = []
    for obstacle_params in params['real_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        real_obstacles.append(faces)
        real_obstacles_vertices.append(vertices)
        ax.add_collection3d(Poly3DCollection(faces, facecolors='grey', linewidths=0.5, edgecolors=[0.1, 0.1, 0.1], alpha=.1))
    return real_obstacles, real_obstacles_vertices

def plot_start_and_end(ax, params):
    ax.scatter(*params['start'], color='green', s=200, label='Start')
    ax.scatter(*params['end'], color='blue', s=1, label='End', marker="*")

def create_polyhedron(vertices, faces_index):
    vertices = np.array(vertices)
    faces = []
    for face_index in faces_index:
        face_points = vertices[face_index]
        faces.append(face_points)

    faces = [np.array(face) for face in faces]
    return faces, vertices.tolist()