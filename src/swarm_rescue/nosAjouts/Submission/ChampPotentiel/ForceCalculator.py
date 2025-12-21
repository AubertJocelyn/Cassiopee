import math
from copy import deepcopy

import numpy as np


class ForceCalculator:
    def __init__(self):
        pass

    def get_consigne(self, robot_pose, aimedPoint, lidar_dist, lidar_angles, vs):

        MAX_DISTANCE = 40.0
        MIN_DISTANCE = 5.0
        K_REPULSIVE = 0.1
        K_ATTRACTIVE = 1.0

        lidar_dist = self.weight_distances(lidar_dist, lidar_angles, vs)

        cos_rays = np.cos(lidar_angles + robot_pose.orientation)
        sin_rays = np.sin(lidar_angles + robot_pose.orientation)

        repulsive_force = np.zeros(2)
        for i in range(len(lidar_dist)):
            if lidar_dist[i] > MAX_DISTANCE:
                continue
            else:
                F = K_REPULSIVE * (1 / max(1, lidar_dist[i]) - MIN_DISTANCE)**2
                Fx = F * cos_rays[i]
                Fy = F * sin_rays[i]
                repulsive_force += np.asarray([Fx, Fy])

        x_drone = robot_pose.position[0]
        y_drone = robot_pose.position[1]
        Fx = K_ATTRACTIVE * (aimedPoint[0] - x_drone)
        Fy = K_ATTRACTIVE * (aimedPoint[1] - y_drone)
        attractive_force = np.asarray([Fx, Fy])
        #print("attractive/repulsive", (attractive_force[0]**2 + attractive_force[1]**2) / (repulsive_force[0]**2 + repulsive_force[1]**2))

        total_force = attractive_force - repulsive_force

        if total_force[0] == 0.0 and total_force[1] == 0.0:
            return {'forward': 0.0, 'lateral': 0.0}

        Fx = total_force[0] / (total_force[0]**2 + total_force[1]**2)**0.5
        Fy = total_force[1] / (total_force[0]**2 + total_force[1]**2)**0.5
        theta = robot_pose.orientation
        forward = Fx * np.cos(theta) + Fy * np.sin(theta)
        lateral = - Fx * np.sin(theta) + Fy * np.cos(theta)

        return {'forward': 1.0*forward, 'lateral': 1.0*lateral, 'grasper': 0}

    def get_v_norm_angle(self, v):
        v_norm = (v[0]**2 + v[1]**2)**0.5
        if v[0] < 10**(-5):
            v_angle = math.copysign(1, v[1])*np.pi/2
        else:
            v_angle = np.arctan(v[1]/v[0])
        return v_norm, v_angle

    def weight_distances(self, lidar_dist, lidar_angles, vs):
        COEFF = 2
        v_norm, v_angle = self.get_v_norm_angle(vs)
        v_norm *= COEFF
        sortie = deepcopy(lidar_dist)
        for i in range(len(lidar_dist)):
            sortie[i] = (max(0, lidar_dist[i] - max(0, v_norm * np.cos(v_angle - lidar_angles[i]))))
        return sortie



