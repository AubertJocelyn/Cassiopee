import math

import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid


class FreeZonesGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution, np.int8)

    def update(self, emptySquaresGrid):
        self.reset_grid()
        grid_width = len(emptySquaresGrid[0])
        grid_height = len(emptySquaresGrid)
        free_zone_length = self.get_free_zone_length(self.resolution)
        offset_center_square = self.get_offset_center_square(self.resolution)
        for i in range(grid_height):
            for j in range(grid_width):
                if emptySquaresGrid[i][j] >= free_zone_length:
                    x_square_center = i - offset_center_square
                    y_square_center = j - offset_center_square
                    self[x_square_center, y_square_center] = 1

    @staticmethod
    def is_cell_empty(value) -> bool:
        VALUE_EMPTY = -2.0
        return value < VALUE_EMPTY

    @staticmethod
    def get_free_zone_length(resolution):
        DRONE_RADIUS = 9
        return 1 + 2 * (1 + math.floor(DRONE_RADIUS / resolution - 1 / 2))

    @staticmethod
    def get_offset_center_square(resolution):
        DRONE_RADIUS = 15
        return 1 + math.floor(DRONE_RADIUS / resolution - 1 / 2)