from collections import deque

import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
from swarm_rescue.simulation.utils.pose import Pose

N_MUR = 0
N_ACCESSIBLE = 1
N_UNEXPLORED = 2
N_RESCUE_CENTER = 3

class RescueCenterGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution, np.int8)

    def update(self, robot_pose, semantic_values):
        if any(data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER for data in semantic_values):
            x_world, y_world = robot_pose.position
            i, j = self._conv_world_to_grid(x_world, y_world)
            self[i, j] = 1


