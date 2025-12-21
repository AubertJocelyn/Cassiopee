from collections import deque

import numpy as np
from tcod import tcod

from swarm_rescue.simulation.utils.pose import Pose

N_MUR = 0
N_UNEXPLORED = 2

class Pathfinder:
    def __init__(self):
        self.safe_position = None
        self.path = None

    def main(self, stateGrid, start_grid, searchValue = 2):
        goal_grid = self.findReachableUnexploredCell(stateGrid, start_grid, searchValue)
        if goal_grid is None:
            start_grid = self.safe_position
            goal_grid = self.findReachableUnexploredCell(stateGrid, start_grid, searchValue)
        self.getPathToDestination(stateGrid, start_grid, goal_grid)
        return self.getLookAheadPoint()

    def findReachableUnexploredCell(self, stateGrid, start_grid, searchValue = 2):
        # grid  : numpy array contenant 0, 1, 2
        # start : (i, j), position de départ (case contenant 1)

        grid_height = len(stateGrid)
        grid_width = len(stateGrid[0])
        visited = np.zeros_like(stateGrid, dtype=bool)

        # BFS
        q = deque()
        q.append(start_grid)
        visited[start_grid] = True

        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # 4-adjacence

        while q:
            i, j = q.popleft()

            # Si on trouve la cible
            if stateGrid[i, j] == searchValue:
                self.safe_position = start_grid
                return (i, j)

            # explorer les voisins
            for di, dj in directions:
                ni, nj = i + di, j + dj

                # vérifier limites
                if 0 <= ni < grid_height and 0 <= nj < grid_width:
                    # cellule franchissable ?
                    if not visited[ni, nj] and stateGrid[ni, nj] != N_MUR:
                        visited[ni, nj] = True
                        q.append((ni, nj))


    def getPathToDestination(self, stateGrid, start_grid, goal_grid):
        if goal_grid is None:
            goal_grid = start_grid

        # tcod attend un tableau de coûts : 0 = bloqué, >0 = coût d'entrer dans la case

        graph = tcod.path.SimpleGraph(cost=stateGrid, cardinal=1, diagonal=1)
        pf = tcod.path.Pathfinder(graph)

        pf.add_root(start_grid)
        path = pf.path_to(goal_grid).tolist()  # liste de (ligne, colonne)

        if path is None:
            self.path = [start_grid]
        elif len(path) == 0:
            self.path = [start_grid]
        else:
            self.path = path

    def getLookAheadPoint(self):
        LOOK_AHEAD_DISTANCE = 4
        if len(self.path) < LOOK_AHEAD_DISTANCE:
            return self.path[0]
        else:
            return self.path[LOOK_AHEAD_DISTANCE - 1]

    def isMapExplored(self,stateGrid):
        if self.safe_position is None:
            return False
        if self.findReachableUnexploredCell(stateGrid, self.safe_position) is None:
            return True
