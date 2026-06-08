from re import S
import numpy as np
from scipy.spatial.transform import Rotation as R

class Trajectory:
    
    def __init__(self, points = []):
        
        self.points = points
        self.n_points = len(points)
        self.nearest_point = 0
        self.last_nearest_ = 0
        
        self.nvec_support_plane = np.asarray([0, -np.sin(np.pi * 40 / 180), np.cos(np.pi * 40 / 180)])  # normal vector of the support plane
    def get_nearest(self, Pose):  # Pose: [x, y, z, qw, qx, qy, qz]
        
        point = np.array([Pose[0], Pose[1], Pose[2]])
        distances = np.linalg.norm(self.points - point, axis=1)
        
        nearest = (np.argmin(distances[self.nearest_point:self.nearest_point + 100 if self.nearest_point + 100 < self.n_points
                                       else self.n_points - 1]) + self.nearest_point) % self.n_points
        self.nearest_point = nearest 
        return nearest
    
    def get_tangent(self, point_index):  
        self.tangent = np.array([self.points[point_index + 1][0] - self.points[point_index][0], 
                       self.points[point_index + 1][1] - self.points[point_index][1],
                       self.points[point_index + 1][2] - self.points[point_index][2]])
        self.tangent = self.tangent / np.linalg.norm(self.tangent)
        return self.tangent
    
    def get_which_side(self, Pose): 
        # positive for the left, negative for the right
        # determine which side the robot is laid on the trajectory
        self.get_nearest(Pose)
        point = np.array([Pose[0], Pose[1], Pose[2]])
        t_ = np.array([self.points[self.nearest_point + 1][0] - self.points[self.nearest_point][0], 
                       self.points[self.nearest_point + 1][1] - self.points[self.nearest_point][1],
                       self.points[self.nearest_point + 1][2] - self.points[self.nearest_point][2]])
        
        r_ = np.array([point[0] - self.points[self.nearest_point][0], 
                       point[1] - self.points[self.nearest_point][1],
                       point[2] - self.points[self.nearest_point][2]])
        
        side = np.sign(np.dot(np.cross(t_, r_), self.nvec_support_plane))

        return side
    
    def get_theta_error(self, Pose):

        R_ = R.from_quat([Pose[4], Pose[5], Pose[6], Pose[3]]).as_matrix()

        vec_head = R_ @ np.array([0, 1, 0])  #
        #vec_head = np.array([2 * Pose[4] * Pose[5] - 2 * Pose[3] * Pose[6],
        #                     Pose[3] * Pose[3] + Pose[5] * Pose[5] - Pose[4] * Pose[4] - Pose[6] * Pose[6],
        #                     2 * Pose[3] * Pose[4] + 2 * Pose[5] * Pose[6]])
        
        self.get_tangent(self.get_nearest(Pose))
        theta_out_of_plane = np.arccos(np.dot(vec_head, self.nvec_support_plane))
        theta_error = np.arccos(np.dot(vec_head, self.tangent) / (np.linalg.norm(vec_head) * np.linalg.norm(self.tangent)))
        return theta_error * theta_out_of_plane
    
    def get_distance_error(self, Pose):
        self.get_nearest(Pose)
        point = np.array([Pose[0], Pose[1], Pose[2]])
        return np.linalg.norm(point - self.points[self.nearest_point])
    
    def path_ellipse_slope(self, rx, ry) -> list:
        t = np.linspace(0, 2 * np.pi, 2000)
        x = rx * np.cos(t) - rx
        y = ry * np.sin(t) * np.cos(np.pi / 9 * 2)
        z = ry * np.sin(t) * np.sin(np.pi / 9 * 2)
        listPath = np.transpose([x, y, z])
        self.points = np.transpose([x, y, z])
        self.n_points = 2000
        return listPath
    
    def path_cubic_spline(self, origin_0 : np.ndarray, R0 : np.ndarray, origin_1 : np.ndarray, R1 : np.ndarray):
        # with given original pose and the destination pose, calculate a 3rd spline as a trajectory
        a_0 = origin_0
        a_1 = R0 @ np.array([0, 1, 0])
        a_2 = (-R1 - 2 * R0) @ np.array([0, 1, 0]) + 3 * origin_1 - 3 * origin_0
        a_3 = (R0 + R1) @ np.array([0, 1, 0]) - 2 * origin_1 + 2 * origin_0

        t = np.linspace(0, 1, 1000)
        x_t = a_0[0] + a_1[0] * t + a_2[0] * t**2 + a_3[0] * t**3
        y_t = a_0[1] + a_1[1] * t + a_2[1] * t**2 + a_3[1] * t**3
        z_t = a_0[2] + a_1[2] * t + a_2[2] * t**2 + a_3[2] * t**3
        self.points = np.transpose([x_t, y_t, z_t])
        self.n_points = 1000


    def get_which_side_tangent(self, Pose): # positive for the left, negative for the right
        self.get_nearest(Pose)
        vec_head = np.array([2 * Pose[4] * Pose[5] - 2 * Pose[3] * Pose[6],
                             Pose[3] * Pose[3] + Pose[5] * Pose[5] - Pose[4] * Pose[4] - Pose[6] * Pose[6],
                             2 * Pose[3] * Pose[4] + 2 * Pose[5] * Pose[6]])
        t_ = self.get_tangent(self.get_nearest(Pose))
        side = np.sign(np.dot(np.cross(t_, vec_head), self.nvec_support_plane))
        return side
        