from collections import deque

import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid
from swarm_rescue.simulation.utils.pose import Pose

N_MUR = 0
N_ACCESSIBLE = 1
N_UNEXPLORED = 2

class StateGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution, np.int8)

    def update(self, freeZonesGrid, obstructuredZonesGrid):
        self.grid = freeZonesGrid.copy()
        grid_height, grid_width = self.grid.shape
        for i in range(grid_height):
            for j in range(grid_width):
                if self[i, j] != N_ACCESSIBLE:
                    if obstructuredZonesGrid[i, j] == 1:
                        self[i, j] = N_MUR
                    else:
                        self[i, j] = N_UNEXPLORED


