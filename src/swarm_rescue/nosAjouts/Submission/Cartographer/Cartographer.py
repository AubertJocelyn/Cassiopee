from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.FreeSquaresGrid import FreeSquaresGrid
from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.FreeZonesGrid import FreeZonesGrid
from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.ObstructuredZonesGrid import ObstructuredZonesGrid
from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.RawMappingGrid import RawMappingGrid
from swarm_rescue.nosAjouts.Submission.Cartographer.Grids.StateGrid import StateGrid


class Cartographer:
    rawMappingGrid: RawMappingGrid
    freeSquaresGrid: FreeSquaresGrid
    freeZonesGrid: FreeZonesGrid
    obstructuredZonesGrid: ObstructuredZonesGrid
    stateGrid: StateGrid
    counter:int

    def __init__(self, self_area_world, resolution):
        self.rawMappingGrid = RawMappingGrid(self_area_world, resolution)
        self.freeSquaresGrid = FreeSquaresGrid(self_area_world, resolution)
        self.freeZonesGrid = FreeZonesGrid(self_area_world, resolution)
        self.obstructuredZonesGrid = ObstructuredZonesGrid(self_area_world, resolution)
        self.stateGrid = StateGrid(self_area_world, resolution)
        self.counter = 0

    def update_map(self, robot_pose, lidar_values):
        self.rawMappingGrid.update(robot_pose, lidar_values["distances"], lidar_values["angles"])
        if self.counter == 0:
            self.reset_counter()
            self.freeSquaresGrid.update(self.rawMappingGrid.show())
            self.freeZonesGrid.update(self.freeSquaresGrid.show())
            self.obstructuredZonesGrid.update(self.rawMappingGrid.show())
            self.stateGrid.update(self.freeZonesGrid.show(), self.obstructuredZonesGrid.show())
        self.counter -= 1

    def reset_counter(self):
        self.counter = 5

    def _conv_grid_to_world(self, x_grid, y_grid):
        return self.stateGrid._conv_grid_to_world(x_grid, y_grid)

    def _conv_world_to_grid(self, x_world, y_world):
        return self.stateGrid._conv_world_to_grid(x_world, y_world)