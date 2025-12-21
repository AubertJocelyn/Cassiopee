import numpy as np
from shiboken6 import Object

from swarm_rescue.nosAjouts.Submission.Rescue.RescueAbilitiesMixin import RescueAbilitiesMixin
from swarm_rescue.nosAjouts.Submission.Rescue.RescueAnalyzer import RescueAnalyzer
from swarm_rescue.nosAjouts.Submission.ChampPotentiel.ForceCalculator import ForceCalculator
from swarm_rescue.nosAjouts.Submission.Cartographer.Cartographer import Cartographer
from swarm_rescue.nosAjouts.Submission.Pathfinder.Pathfinder import Pathfinder
from swarm_rescue.nosAjouts.Submission.Printer.Printer import Printer
from swarm_rescue.nosAjouts.Submission.States.StateManagementAbilitiesMixin import StateManagementAbilitiesMixin
from swarm_rescue.nosAjouts.Submission.States.StateManager import StateManager
from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract


class OurDrone(DroneAbstract, RescueAbilitiesMixin, StateManagementAbilitiesMixin):
    x_drone: float
    y_drone: float
    angle_drone: float
    lidar_values: dict
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.cartographer = Cartographer(resolution=8, self_area_world=self.size_area)
        self.pathfinder = Pathfinder()
        self.forceCalculator = ForceCalculator()
        self.printer = Printer()
        self.rescueAnalyzer = RescueAnalyzer()
        self.stateManager = StateManager(self)
        self.aimed_point_world = None
        self.isCarryingWounded = False

    def define_message_for_all(self):
        pass

    def get_pose(self):
        return Pose(np.asarray([self.x_drone, self.y_drone]), self.angle_drone)

    def control(self) -> CommandsDict:
        print(type(self.stateManager.state))
        self.update_lidar_and_pose()
        self.cartographer.update_map(robot_pose=self.get_pose(), lidar_values=self.lidar_values)
        self.rescueAnalyzer.update(self.get_pose(), self.semantic_values())
        self.printer.update(self.cartographer.stateGrid, self.pathfinder.path)
        self.updateState()
        return self.stateManager.state.get_command()

    def update_lidar_and_pose(self):
        self.x_drone, self.y_drone = self.measured_gps_position()
        self.angle_drone = self.measured_compass_angle()
        self.lidar_values = {
            "angles": self.lidar().ray_angles.copy(),
            "distances": self.lidar().get_sensor_values().copy()
        }

    def getCommandForceCalculator(self):
        return self.forceCalculator.get_consigne(self.get_pose(), self.aimed_point_world, self.lidar_values["distances"], self.lidar_values["angles"], self.measured_velocity())

    def setRescueCenterPosition(self):
        i, j = self.cartographer._conv_world_to_grid(self.rescueAnalyzer.x_center, self.rescueAnalyzer.y_center)
        self.cartographer.stateGrid[i, j] = 3

    def setWoundedPosition(self):
        i, j = self.cartographer._conv_world_to_grid(self.rescueAnalyzer.x_wounded, self.rescueAnalyzer.y_wounded)
        self.cartographer.stateGrid[i, j] = 4
        
    def set_local_point_world_wounded(self):
        self.aimed_point_world = (self.rescueAnalyzer.x_wounded, self.rescueAnalyzer.y_wounded)

    def getCurrentAngle(self):
        return self.angle_drone

    ########################################################################################################################
    #Cartographer and Pathfinder
    def set_aimed_point_world_rescue_center(self):
        self.aimed_point_world = self.get_aimed_point_world(3)

    def set_aimed_point_world_unexplored(self):
        self.aimed_point_world = self.get_aimed_point_world(2)

    def set_aimed_point_world_wounded(self):
        self.aimed_point_world = self.get_aimed_point_world(4)

    def get_aimed_point_world(self, value):
        x_drone_grid, y_drone_grid = self.cartographer._conv_world_to_grid(self.x_drone, self.y_drone)
        aimed_x_grid, aimed_y_grid = self.pathfinder.main(self.cartographer.stateGrid.show(), (x_drone_grid, y_drone_grid), value)
        return self.cartographer._conv_grid_to_world(aimed_x_grid, aimed_y_grid)

    def isMapExplored(self):
        return self.pathfinder.isMapExplored(self.cartographer.stateGrid.show())

