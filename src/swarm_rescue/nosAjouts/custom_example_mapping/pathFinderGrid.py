import numpy as np
import tcod.path

from swarm_rescue.simulation.utils.grid import Grid


class PathFinderGrid(Grid):
    path: list
    grid: np.ndarray

    def __init__(self, size_area_world, resolution: float):
        super().__init__(size_area_world=size_area_world, resolution=resolution)

    def update(self, map_grid, start_world = (-50.0, 0), goal_world = (-472, 174), **kwargs):
        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.grid_map = map_grid
        start_grid = self._conv_world_to_grid(start_world[0], start_world[1])
        goal_grid = self._conv_world_to_grid(goal_world[0], goal_world[1])
        # tcod attend un tableau de coûts : 0 = bloqué, >0 = coût d'entrer dans la case

        graph = tcod.path.SimpleGraph(cost=map_grid, cardinal=1, diagonal=1)
        pf = tcod.path.Pathfinder(graph)

        pf.add_root(start_grid)
        path = pf.path_to(goal_grid).tolist()  # liste de (ligne, colonne)

        for i, j in path:
            self[i, j] = 1
