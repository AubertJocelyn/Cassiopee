from copy import copy
from logging import exception

import numpy as np
from collections import deque

from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap

from swarm_rescue.simulation.utils.grid import Grid
from swarm_rescue.simulation.utils.pose import Pose

N_MUR = 0
N_ACCESSIBLE = 1
N_UNEXPLORED = 2

class ClosestUnexploredGird(Grid):
    map_grid: np.ndarray
    drone_radius_on_grid: int
    grid: np.ndarray

    def __init__(self,
                 size_area_world,
                 resolution: float):
        super().__init__(size_area_world=size_area_world,
                         resolution=resolution)
        self.grid = self.grid = np.zeros((self.x_max_grid, self.y_max_grid), dtype=np.int8)

    def update(self, map_possible_positions, map_impossible_positions, **kwargs):
        self.grid = map_possible_positions.copy()
        grid_height, grid_width = self.grid.shape
        for i in range(grid_height):
            for j in range(grid_width):
                if self[i, j] != N_ACCESSIBLE:
                    if map_impossible_positions[i, j] == 1:
                        self[i, j] = N_MUR
                    else:
                        self[i, j] = N_UNEXPLORED

    def find_nearest_unexplored(self, start_world):
        start_grid = self._conv_world_to_grid(start_world[0], start_world[1])
        
        #grid  : numpy array contenant 0, 1, 2
        #start : (i, j), position de départ (case contenant 1)
        

        grid_height, grid_width = self.grid.shape
        visited = np.zeros_like(self.grid, dtype=bool)

        # BFS
        q = deque()
        q.append(start_grid)
        visited[start_grid] = True

        directions = [(1,0), (-1,0), (0,1), (0,-1)]  # 4-adjacence

        while q:
            i, j = q.popleft()


            # Si on trouve la cible
            if self.grid[i, j] == N_UNEXPLORED:
                x, y = self._conv_grid_to_world(i, j)
                return Pose(np.asarray(np.array([x, y])))

            # explorer les voisins
            for di, dj in directions:
                ni, nj = i + di, j + dj

                # vérifier limites
                if 0 <= ni < grid_height and 0 <= nj < grid_width:
                    # cellule franchissable ?
                    if not visited[ni, nj] and self.grid[ni, nj] != N_MUR:
                        visited[ni, nj] = True
                        q.append((ni, nj))

        return None

    def reposition_drone_in_accessible_area(self, x_grid, y_grid):
        if self[x_grid, y_grid] == N_MUR:
            pass