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
        self.dico_grids["PathFinderGrid"] = PathFinderGrid(resolution=resolution, size_area_world=size_area_world)
        self.dico_grids["PossiblePositions"] = PossiblePositionsGrid(resolution=resolution, size_area_world=size_area_world)

    def update(self, **kwargs):
        for key in self.dico_grids.keys():
            self.dico_grids[key].update(map_grid = self.dico_grids["OccupancyGrid"].show(), **kwargs)

    def show(self):
        self.display(truncate(self.dico_grids["OccupancyGrid"].zoomed(), -20, 20),
                     title="zoomed occupany grid troncated")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["DroneGrid"].zoomed(),
                     title="zoomed drone + occupancy grid")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["PossiblePositions"].zoomed(),
                     title="possible positions")
        self.display(self.dico_grids["OccupancyGrid"].zoomed(),
                     self.dico_grids["PossiblePositions"].zoomed(),
                     title="zoomed possible positions + occupancy grid")

    @staticmethod
    def display(grid_to_display: np.ndarray, grid_customisable: np.ndarray = None, title: str = "grid"):
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