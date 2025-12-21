import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.CustomGrid import CustomGrid


class FreeSquaresGrid(CustomGrid):
    def __init__(self, size_area_world, resolution):
        super().__init__(size_area_world, resolution, np.int8)

    def update(self, rawMappingGrid):
        self.reset_grid()
        grid_width = len(rawMappingGrid[0])
        grid_height = len(rawMappingGrid)
        for i in range(grid_height):
            for j in range(grid_width):
                cell_value = rawMappingGrid[i, j]
                if not self.is_cell_free(cell_value):
                    self[i, j] = 0
                elif (i == 0 or j == 0):
                    self[i, j] = 1
                else:
                    self[i,j] = 1 + min(
                        self[i - 1, j - 1],
                        self[i - 1, j],
                        self[i, j - 1]
                    )

    @staticmethod
    def is_cell_free(value) -> bool:
        VALUE_FREE = -2.0
        return value < VALUE_FREE

