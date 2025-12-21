import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid
from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR


class RawMappingGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution)

    def update(self, robot_pose, lidar_dist, lidar_angles):
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

        # Compute cos and sin of the absolute angle of the lidar
        cos_rays = np.cos(lidar_angles + robot_pose.orientation)
        sin_rays = np.sin(lidar_angles + robot_pose.orientation)

        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        # For empty zones
        # points_x and point_y contains the border of detected empty zone
        # We use a value a little bit less than LIDAR_DIST_CLIP because of the
        # noise in lidar
        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        # All values of lidar_dist_empty_clip are now <= max_range
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        points_x = robot_pose.position[0] + np.multiply(lidar_dist_empty_clip,
                                                  cos_rays)
        points_y = robot_pose.position[1] + np.multiply(lidar_dist_empty_clip,
                                                  sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(robot_pose.position[0], robot_pose.position[1],
                                      pt_x, pt_y,
                                      EMPTY_ZONE_VALUE)

        # For obstacle zones, all values of lidar_dist are < max_range
        select_collision = lidar_dist < max_range

        points_x = robot_pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = robot_pose.position[1] + np.multiply(lidar_dist, sin_rays)

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)

        # the current position of the drone is free !
        self.add_points(robot_pose.position[0], robot_pose.position[1], FREE_ZONE_VALUE)

        # threshold values
        self.grid = self.clipped(THRESHOLD_MIN, THRESHOLD_MAX)