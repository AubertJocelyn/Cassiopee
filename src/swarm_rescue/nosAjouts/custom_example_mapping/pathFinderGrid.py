import numpy as np
import tcod.path

from swarm_rescue.simulation.utils.grid import Grid


class PathFinderGrid(Grid):
    path: list
    grid: np.ndarray

    def __init__(self, size_area_world, resolution: float):
        super().__init__(size_area_world=size_area_world, resolution=resolution)
        self.path = None

    def update(self, map_closest_unexplored, start_world = (-50.0, 0), goal_world = (-472, 174), **kwargs):
        self.grid = np.zeros((self.x_max_grid, self.y_max_grid), dtype=np.int8)
        self.map_closest_unexplored = map_closest_unexplored
        start_grid = self._conv_world_to_grid(start_world[0], start_world[1])
        if goal_world is None:
            goal_grid = start_grid
        else:
            #goal_grid = self._conv_world_to_grid(goal_world[0], goal_world[1])
            x_drone = goal_world.position[0]
            y_drone = goal_world.position[1]
            x, y = self._conv_world_to_grid(x_drone, y_drone)
            goal_grid = (x, y)

        # tcod attend un tableau de coûts : 0 = bloqué, >0 = coût d'entrer dans la case

        graph = tcod.path.SimpleGraph(cost=map_closest_unexplored, cardinal=1, diagonal=1)
        pf = tcod.path.Pathfinder(graph)

        pf.add_root(start_grid)
        path = pf.path_to(goal_grid).tolist()  # liste de (ligne, colonne)

        for i, j in path:
            self[i, j] = 1
        if path is None:
            self.path = [start_grid]
        elif len(path) == 0:
            self.path = [start_grid]
        else:
            self.path = path

    def get_next_point(self):
        if self.path is None:
            return (0.0, 0.0)
        else:
            path_world = self.convert_path_from_grid_to_world(self.path)
        if len(self.path) < 4:
            return path_world[0]
        else:
            return path_world[3]

    def convert_path_from_grid_to_world(self, path_grid):
        path_world = []
        for x_grid, y_grid in path_grid:
            path_world.append(self._conv_grid_to_world(x_grid, y_grid))
        return path_world

    def line_of_sight(self, pi, pj):
        VALUE_FULL = 0.0
        line = self.Bresenham(pi[0], pi[1], pj[0], pj[1]).T
        for i, j in line:
            if self.map_closest_unexplored[i, j] > VALUE_FULL:
                return False
        return True
