import numpy as np

from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.utils import math as omni_math
from omni.isaac import _sim as omni_sim
from omni.isaac.debug_draw import _debug_draw as omni_draw

class DrawSimulationFeatures: 

    def __init__(self, kit):
        self.sim = kit
        self.draw = omni_draw.acquire_debug_draw_interface()

    def draw_cov_ellipse(self, mu, cov, color):
        U, s, V = np.linalg.svd(cov)
        a, b = s[0], s[1]
        vx, vy = V[0, 0], V[0, 1]
        theta = np.arctan2(vy, vx)
        phi = np.arange(0, 2 * np.pi, np.pi / 50)
        ellipse_points = np.array([3*np.sqrt(a)*np.cos(phi), 3*np.sqrt(b)*np.sin(phi)])
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        rotated_ellipse_points = R @ ellipse_points + np.array(mu)[:, None]

        # Draw the ellipse in Isaac Sim
        for i in range(len(rotated_ellipse_points.T) - 1):
            start_point = omni_math.Vec3(rotated_ellipse_points[0, i], rotated_ellipse_points[1, i], 0)
            end_point = omni_math.Vec3(rotated_ellipse_points[0, i + 1], rotated_ellipse_points[1, i + 1], 0)
            self.draw.add_line(start_point, end_point, color)


    def draw_traj_and_pred(self, X, P):
        self.draw_cov_ellipse(X[0:2], P[0:2, 0:2], (1.0, 0.0, 0.0)) # red

        start_point = omni_math.Vec3(X[0], X[1], 0.0)
        end_point = omni_math.Vec3(X[0] + 1.0, X[1] + 1.0, 0.0)
        self.draw.add_line(start_point, end_point, (0.0, 1.0, 0.0)) # green

        self.kit.update_renderer_sync() 
        

    def draw_traj_and_map(self, X, last_X, P, t):
        
        start_point = omni_math.Vec3(last_X[0], last_X[1], 0.0)
        end_point = omni_math.Vec3(X[0], X[1], 0.0)
        self.draw.add_line(start_point, end_point, (0.0, 0.0, 1.0)) # blue

        self.draw.add_point(omni_math.Vec3(X[0], X[1], 0.0), (0.0, 0.0, 0.1))

        # Draw the covariance ellipse
        if t == 0:
            for k in range(6):
                self.draw_cov_ellipse(
                    X[3 + k * 2:3 + k * 2 + 2], P[3 + k * 2:3 + 2 * k + 2,
                                                  3 + 2 * k:3 + 2 * k + 2], (1.0, 0.0, 0.0)) # red
                
        else:
            for k in range(6):
                self.draw_cov_ellipse(
                    X[3 + k * 2:3 + k * 2 + 2], P[3 + 2 * k:3 + 2 * k + 2,
                                                  3 + 2 * k:3 + 2 * k + 2], (0.0, 1.0, 0.0)) # green

        self.kit.update_renderer_sync()