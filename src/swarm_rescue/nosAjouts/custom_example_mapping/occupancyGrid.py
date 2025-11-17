import pathlib
import sys
from typing import Type

import cv2
import numpy as np
from typing_extensions import override

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR
from swarm_rescue.simulation.utils.grid import Grid
from swarm_rescue.simulation.utils.pose import Pose

class OccupancyGrid(Grid):
    """Simple occupancy grid"""

    def __init__(self,
                 size_area_world,
                 resolution: float,
                 lidar):
        super().__init__(size_area_world=size_area_world,
                         resolution=resolution)

        self.size_area_world = size_area_world
        self.resolution = resolution

        self.lidar = lidar

        self.x_max_grid: int = int(self.size_area_world[0] / self.resolution
                                   + 0.5)
        self.y_max_grid: int = int(self.size_area_world[1] / self.resolution
                                   + 0.5)

        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.zoomed_grid = np.empty((self.x_max_grid, self.y_max_grid))

    def update_grid(self, pose: Pose):
        """
        Bayesian map update with new observation
        lidar : lidar data
        pose : corrected pose in world coordinates
        """
        EVERY_N = 1
        LIDAR_DIST_CLIP = 40.0
        EMPTY_ZONE_VALUE = -0.602
        OBSTACLE_ZONE_VALUE = 2.0
        FREE_ZONE_VALUE = -4.0
        THRESHOLD_MIN = -40
        THRESHOLD_MAX = 40

        lidar_dist = self.lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = self.lidar.ray_angles[::EVERY_N].copy()

        # Compute cos and sin of the absolute angle of the lidar
        cos_rays = np.cos(lidar_angles + pose.orientation)
        sin_rays = np.sin(lidar_angles + pose.orientation)

        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        # For empty zones
        # points_x and point_y contains the border of detected empty zone
        # We use a value a little bit less than LIDAR_DIST_CLIP because of the
        # noise in lidar
        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        # All values of lidar_dist_empty_clip are now <= max_range
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        points_x = pose.position[0] + np.multiply(lidar_dist_empty_clip,
                                                  cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist_empty_clip,
                                                  sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1],
                                      pt_x, pt_y,
                                      EMPTY_ZONE_VALUE)

        # For obstacle zones, all values of lidar_dist are < max_range
        select_collision = lidar_dist < max_range

        points_x = pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist, sin_rays)

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)

        # the current position of the drone is free !
        self.add_points(pose.position[0], pose.position[1], FREE_ZONE_VALUE)

        # threshold values
        self.grid = self.clip_grid(self.grid, THRESHOLD_MIN, THRESHOLD_MAX)
        self.zoomed_grid = self.get_zoomed_grid(self.grid)

    @staticmethod
    def clip_grid(grid, a, d, b = -1, c = 1):
        grid1 = np.clip(grid, a, b)
        grid2 = np.clip(grid, c, d)
        return grid1 + grid2

    # compute zoomed grid for displaying
    def get_zoomed_grid(self, grid):
        zoomed_grid = grid.copy()
        new_zoomed_size = (int(self.size_area_world[1] * 0.5),
                           int(self.size_area_world[0] * 0.5))
        zoomed_grid = cv2.resize(zoomed_grid, new_zoomed_size,
                                      interpolation=cv2.INTER_NEAREST)
        return zoomed_grid

    def get_grid_map(self):
        return self.grid

    def get_grid_drone(self, robot_pose: Pose):
        radius_drone = 15
        x_drone = robot_pose.position[0]
        y_drone = robot_pose.position[1]
        grid = np.zeros((self.x_max_grid, self.y_max_grid))
        for i in range(2*radius_drone):
            for j in range(2 * radius_drone):
                pt1_x, pt1_y = (self._conv_world_to_grid(x_drone + i,y_drone + j))
                grid[int(pt1_x)][int(pt1_y)] = 1.0
        return grid


    def get_grid_drone_zoomed(self, robot_pose: Pose):
        return self.get_zoomed_grid(self.get_grid_drone(robot_pose))

    def print_grid(self, robot_pose: Pose):
        self.display(self.grid,
                          self.get_grid_drone(robot_pose),
                          title="occupancy grid")
        self.display(self.zoomed_grid, self.get_grid_drone_zoomed(robot_pose),
                          title="zoomed occupancy grid")
    @override
    def display(self, grid_to_display: np.ndarray, grid_customisable: np.ndarray = None, title: str = "grid"):
        img = grid_to_display.T
        img = img - img.min()
        img = img / img.max() * 255
        img = np.uint8(img)
        img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

        ###ajout chat GPT pour afficher le drone de manière consistante
        mask = (grid_customisable.T != 0)
        img_color[mask] = [255, 255, 255]

        cv2.imshow(title, img_color)
        cv2.waitKey(1)
