import cv2
import numpy as np

from swarm_rescue.nosAjouts.custom_example_mapping.droneGrid import DroneGrid
from swarm_rescue.nosAjouts.custom_example_mapping.pathFinderGrid import PathFinderGrid
from swarm_rescue.nosAjouts.custom_example_mapping.possiblePositionsGrid import PossiblePositionsGrid
from swarm_rescue.simulation.utils.grid import truncate
from swarm_rescue.nosAjouts.custom_example_mapping.occupancyGrid import OccupancyGrid
from swarm_rescue.simulation.utils.pose import Pose


class Printer:
    dico_grids: dict
    def __init__(self, **kwargs):
        self.dico_grids = {}
        resolution = kwargs.get("resolution")
        size_area_world = kwargs.get("size_area_world")
        self.dico_grids["DroneGrid"] = DroneGrid(resolution=resolution, size_area_world=size_area_world)
        self.dico_grids["OccupancyGrid"] = OccupancyGrid(lidar=kwargs.get("lidar"), resolution=resolution, size_area_world=size_area_world)
        self.dico_grids["PossiblePositions"] = PossiblePositionsGrid(resolution=resolution, size_area_world=size_area_world)
        self.dico_grids["PathFinderGrid"] = PathFinderGrid(resolution=resolution, size_area_world=size_area_world)


    def update(self, **kwargs):
        for key in self.dico_grids.keys():
            map_possible_positions = self.dico_grids["PossiblePositions"].show()
            map_grid = self.dico_grids["OccupancyGrid"].show()
            goal_world =  kwargs.get("robot_pose").position
            self.dico_grids[key].update(map_grid = map_grid, map_possible_positions=map_possible_positions, goal_world = goal_world, **kwargs)

    def show(self):
        self.display(truncate(self.dico_grids["OccupancyGrid"].zoomed(), -20, 20),
                     title="zoomed occupany grid troncated")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["DroneGrid"].zoomed(),
                     title="zoomed drone + occupancy grid")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["PossiblePositions"].zoomed(),
                     title="zoomed possible positions + occupancy grid")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["PathFinderGrid"].zoomed(),
                     title="zoomed pathfinder + occupancy grid")

    @staticmethod
    def display(grid_to_display: np.ndarray, grid_customisable: np.ndarray = None, title: str = "grid"):
        if grid_customisable is None:
            grid_customisable = np.zeros_like(grid_to_display)

        img = grid_to_display.T
        img = img - img.min()
        img = img / img.max() * 255
        img = np.uint8(img)
        img_color = cv2.applyColorMap(src=img, colormap=cv2.COLORMAP_JET)

        ###ajout chat GPT pour afficher le drone de manière consistante
        mask = (grid_customisable.T != 0)
        img_color[mask] = [255, 255, 255]

        cv2.imshow(title, img_color)
        cv2.waitKey(1)