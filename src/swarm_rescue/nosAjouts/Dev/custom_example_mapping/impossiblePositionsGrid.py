import math

import numpy as np

from swarm_rescue.simulation.utils.grid import Grid


class ImpossiblePositionsGrid(Grid):
    map_grid: np.ndarray
    drone_radius_on_grid: int
    grid: np.ndarray

    def __init__(self,
                 size_area_world,
                 resolution: float):
        super().__init__(size_area_world=size_area_world,
                         resolution=resolution)
        self.drone_radius_on_grid = self.get_drone_radius_on_grid(resolution)

    def update(self, map_grid, **kwargs):
        ###code originel optimisé par chatGPT
        self.map_grid = map_grid
        H, W = map_grid.shape

        full = self.get_full_cell_grid(map_grid)
        radius = self.drone_radius_on_grid

        # padding autour de la grille
        padded = np.pad(full, radius + 1)

        # fenêtre pour le halo
        kernel_size = 2 * radius + 3

        # convolution logique via sliding windows
        from numpy.lib.stride_tricks import sliding_window_view
        windows = sliding_window_view(padded, (kernel_size, kernel_size))

        # si une fenêtre contient au moins un True -> 1
        self.grid = (windows.any(axis=(-1, -2))).astype(np.int8)

    @staticmethod
    def get_full_cell_grid(grid) -> bool:
        VALUE_FULL = 2.0
        return grid > VALUE_FULL

    @staticmethod
    def get_drone_radius_on_grid(resolution):
        DRONE_RADIUS = 1 * 15
        return 1 + math.floor(DRONE_RADIUS / resolution - 1 / 2)
