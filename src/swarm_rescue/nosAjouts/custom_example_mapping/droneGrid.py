import numpy as np

from swarm_rescue.simulation.utils.grid import Grid


class DroneGrid(Grid):
    def __init__(self, size_area_world, resolution: float):
        super().__init__(size_area_world=size_area_world, resolution=resolution)

    def update(self, robot_pose, **kwargs):
        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        RADIUS_DRONE = 15
        x_drone = robot_pose.position[0]
        y_drone = robot_pose.position[1]
        for i in range(2 * RADIUS_DRONE):
            for j in range(2 * RADIUS_DRONE):
                pt1_x, pt1_y = (self._conv_world_to_grid(x_drone + i, y_drone + j))
                self[int(pt1_x), int(pt1_y)] = 1.0