
import numpy as np
import math

from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline

class Trajectory:
    
    def __init__(self, points = []):
        
        self.points = points  # self.points refers to the sample points of the target path
        self.n_points = len(points)
        self.nearest_point = 0
        self.last_nearest_ = 0
        
    def get_nearest(self, Pose):  # Pose: [x, y, z, qw, qx, qy, qz]
        
        point = np.array([Pose[0], Pose[1]])
        distances = np.linalg.norm(self.points - point, axis=1)
        
        nearest = (np.argmin(distances[self.nearest_point:self.nearest_point + 100 if self.nearest_point + 100 < self.n_points
                                       else self.n_points - 1]) + self.nearest_point) % self.n_points
        self.nearest_point = nearest 
        return nearest
    
    def get_tangent(self, point_index):  
        self.tangent = np.array([self.points[point_index + 1][0] - self.points[point_index][0], 
                       self.points[point_index + 1][1] - self.points[point_index][1]])
        self.tangent = self.tangent / np.linalg.norm(self.tangent)
        return self.tangent
    
    def get_which_side(self, Pose): # positive for the left, negative for the right
        self.get_nearest(Pose)
        point = np.array([Pose[0], Pose[1]])
        t_ = np.array([self.points[self.nearest_point + 1][0] - self.points[self.nearest_point][0], 
                       self.points[self.nearest_point + 1][1] - self.points[self.nearest_point][1]])
        r_ = np.array([point[0] - self.points[self.nearest_point][0], point[1] - self.points[self.nearest_point][1]])
        side = np.sign(t_[0] * r_[1] - t_[1] * r_[0])
        return side
    
    def get_theta_error(self, Pose):
        point = np.array([Pose[0], Pose[1]])

        vec_head = np.array([-2 * Pose[3] * Pose[6], Pose[3] * Pose[3] - Pose[6] * Pose[6]])  #
        self.get_tangent(self.get_nearest(Pose))
        theta_error = np.arccos(np.dot(vec_head, self.tangent) / (np.linalg.norm(vec_head) * np.linalg.norm(self.tangent)))
        return theta_error
    
    def get_distance_error(self, Pose):
        self.get_nearest(Pose)
        point = np.array([Pose[0], Pose[1]])
        return np.linalg.norm(point - self.points[self.nearest_point])
    
    def get_which_side_tangent(self, Pose): # positive for the left, negative for the right
        self.get_nearest(Pose)
        vec_head = np.array([-2 * Pose[3] * Pose[6], Pose[3] * Pose[3] - Pose[6] * Pose[6]])  #
        t_ = self.get_tangent(self.get_nearest(Pose))
        side = np.sign(t_[0] * vec_head[1] - t_[1] * vec_head[0])
        return side
    
    def path_8_shape(self) -> list:
        t = np.linspace(0, 2 * np.pi, 2000)
        x = 600 * np.cos(t) / (1 + np.sin(t) * np.sin(t)) - 600
        y = 600 * np.cos(t) * np.sin(t) / (1 + np.sin(t) * np.sin(t))
        self.points = np.transpose([x, y])
        return self.points
    
    def path_ellipse(self, rx, ry, cx, cy) -> list:
        t = np.linspace(0, 2 * np.pi, 2000)
        x = rx * np.cos(t) + cx  #rx refers to radius towards x-orientation
        y = ry * np.sin(t) + cy  #cx refers to x of center
        self.points = np.transpose([x, y])
        self.n_points = 2000
        return self.points
    
    def path_besele_curve(self, sample_points : list) -> list:
        n = len(sample_points)
        t = np.linspace(0, 1, 2000)
        x = np.zeros(2000)
        y = np.zeros(2000)
        i = 0
        for point in sample_points:
            x1 = point[0]
            y1 = point[1]
            x = x + x1 * math.comb(n, i) * (1 - t) ** (n - 1 - i) * t ** i
            y = y + y1 * math.comb(n, i) * (1 - t) ** (n - 1 - i) * t ** i
            i = i + 1
        self.points = np.transpose([x, y])
        return self.points

    def path_cubic_spline(self, sample_points : list) -> list:
    
        n = len(sample_points)
        t_sample = np.linspace(0, 1, n)
        t = np.linspace(0, 1, 2000)
        
        cubicspline = CubicSpline(t_sample, sample_points)
        interp = cubicspline(t)
        self.points = interp
        return self.points

    def path_broken_line(self, sample_points : list) -> list:
        n = len(sample_points)
        x = np.asarray([])
        y = np.asarray([])
        for i in range(n - 1):
            x_0 = sample_points[i][0]
            y_0 = sample_points[i][1]
            
            x_1 = sample_points[i + 1][0]
            y_1 = sample_points[i + 1][1]

            t_i = np.linspace(0, 1, 300, endpoint=False)
            x_i = x_0 + (x_1 - x_0) * t_i
            y_i = y_0 + (y_1 - y_0) * t_i

            x = np.r_[x, x_i]
            y = np.r_[y, y_i]
            
        self.points = np.transpose([x, y])
        return self.points