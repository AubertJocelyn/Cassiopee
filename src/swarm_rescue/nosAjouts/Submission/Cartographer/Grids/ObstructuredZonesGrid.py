import math

import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid


class ObstructuredZonesGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution, np.int8)

    def update(self, rawMappingGrid):
        self.reset_grid()
        ###code originel optimisé par chatGPT

        full = self.get_obstructured_cell_grid(rawMappingGrid)
        radius = self.get_drone_radius_on_grid(self.resolution)

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
    def get_obstructured_cell_grid(grid) -> bool:
        VALUE_OBSTRUCTURED = 2.0
        return grid > VALUE_OBSTRUCTURED

    @staticmethod
    def get_drone_radius_on_grid(resolution):
        DRONE_RADIUS = 1 * 15
        return 1 + math.floor(DRONE_RADIUS / resolution - 1 / 2)