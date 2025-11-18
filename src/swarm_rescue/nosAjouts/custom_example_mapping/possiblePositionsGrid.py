import math

import numpy as np

from swarm_rescue.simulation.utils.grid import Grid


class PossiblePositionsGrid(Grid):
    map_grid: np.ndarray
    square_length: int
    offset_center_square: int
    empty_zone_length: np.ndarray
    grid: np.ndarray
    def __init__(self,
                 size_area_world,
                 resolution: float):
        super().__init__(size_area_world=size_area_world,
                         resolution=resolution)

        self.empty_zone_length = self.get_empty_zone_length(resolution)
        self.offset_center_square = self.get_offset_center_square(resolution)


    def update(self, map_grid, **kwargs):
        self.map_grid = map_grid
        self.grid_auxiliaire = np.zeros_like(map_grid, dtype=np.int8)
        self.grid = np.zeros_like(map_grid, dtype=np.int8)
        self.set_grid_auxiliaire()
        self.set_grid_positions()

    @staticmethod
    def is_cell_empty(value) -> bool:
        VALUE_EMPTY = -2.0
        return value < VALUE_EMPTY

    @staticmethod
    def get_empty_zone_length(resolution):
        DRONE_RADIUS = 15
        return 1 + 2 * (1 + math.floor(DRONE_RADIUS / resolution - 1 / 2))

    @staticmethod
    def get_offset_center_square(resolution):
        DRONE_RADIUS = 15
        return 1 + math.floor(DRONE_RADIUS / resolution - 1 / 2)

    def set_grid_auxiliaire(self):
        grid_width = len(self.map_grid[0])
        grid_height = len(self.map_grid)
        for i in range(grid_height):
            for j in range(grid_width):
                cell_value = self.map_grid[i, j]
                if not self.is_cell_empty(cell_value):
                    self.grid_auxiliaire[i, j] = 0
                elif (i == 0 or j == 0):
                    self.grid_auxiliaire[i, j] = 1
                else:
                    self.grid_auxiliaire[i,j] = 1 + min(
                        self.grid_auxiliaire[i - 1, j - 1],
                        self.grid_auxiliaire[i - 1, j],
                        self.grid_auxiliaire[i, j - 1]
                    )

    def get_auxiliaire_grid(self):
        return self.grid_auxiliaire

    def set_grid_positions(self):
        grid_width = len(self.map_grid[0])
        grid_height = len(self.map_grid)
        for i in range(grid_height):
            for j in range(grid_width):
                if self.grid_auxiliaire[i][j] >= self.empty_zone_length:
                    x_square_center = i - self.offset_center_square
                    y_square_center = j - self.offset_center_square
                    self[x_square_center, y_square_center] = 1




