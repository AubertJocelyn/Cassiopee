import numpy as np


class ForceCalculator:
    def __init__(self, lidar):
        self.lidar = lidar
    def get_consigne(self, robot_pose, goal_world):
        MAX_DISTANCE = 40.0
        MIN_DISTANCE = 5.0
        K_REPULSIVE = 0.1
        K_ATTRACTIVE = 1.0

        lidar_dist = self.lidar.get_sensor_values().copy()
        lidar_angles = self.lidar.ray_angles.copy()

        cos_rays = np.cos(lidar_angles + robot_pose.orientation)
        sin_rays = np.sin(lidar_angles + robot_pose.orientation)

        repulsive_force = np.zeros(2)
        for i in range(len(lidar_dist)):
            if lidar_dist[i] > MAX_DISTANCE:
                continue
            else:
                F = K_REPULSIVE * (1 / lidar_dist[i] - MIN_DISTANCE)**2
                Fx = F * cos_rays[i]
                Fy = F * sin_rays[i]
                repulsive_force += np.asarray([Fx, Fy])

        x_drone = robot_pose.position[0]
        y_drone = robot_pose.position[1]
        Fx = K_ATTRACTIVE * (goal_world[0] - x_drone)
        Fy = K_ATTRACTIVE * (goal_world[1] - y_drone)
        attractive_force = np.asarray([Fx, Fy])
        print("attractive/repulsive", (attractive_force[0]**2 + attractive_force[1]**2) / (repulsive_force[0]**2 + repulsive_force[1]**2))

        total_force = attractive_force - repulsive_force

        Fx = total_force[0] / (total_force[0]**2 + total_force[1]**2)**0.5
        Fy = total_force[1] / (total_force[0]**2 + total_force[1]**2)**0.5
        theta = robot_pose.orientation
        forward = Fx * np.cos(theta) + Fy * np.sin(theta)
        lateral = - Fx * np.sin(theta) + Fy * np.cos(theta)

        return {'forward': 1.0*forward, 'lateral': 1.0*lateral}


