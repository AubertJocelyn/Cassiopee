"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import pathlib
import sys
from typing import Type

import numpy as np

from swarm_rescue.nosAjouts.Dev.ChampPotentiel.ForceCalculator import ForceCalculator
from swarm_rescue.nosAjouts.Dev.custom_example_mapping.printer import Printer

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.maps.walls_medium_02 import add_walls, add_boxes
from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR
from swarm_rescue.simulation.utils.misc_data import MiscData
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract

class MyDroneMapping(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        resolution = 8
        size_area_world = self.size_area
        self.printer_ = Printer(lidar = self.lidar(), resolution=resolution, size_area_world=self.size_area)
        self.iteration: int = 0
        self.estimated_pose = Pose()
        self.force_calculator = ForceCalculator(self.lidar())

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self) -> CommandsDict:
        """
        We only send a command to do nothing
        """

        # increment the iteration counter
        self.iteration += 1
        self.estimated_pose = Pose(np.asarray(self.measured_gps_position()),
                                   self.measured_compass_angle())
        # self.estimated_pose = Pose(np.asarray(self.true_position()),
        #                            self.true_angle())

        self.printer_.update(robot_pose=self.estimated_pose)
        angle_cons = self.get_further_angle(self.lidar())
        if self.iteration % 30 == 0:
            self.printer_.show()

        command = self.force_calculator.get_consigne(self.estimated_pose, self.printer_.get_goal_world())
        print("command", command)
        return command

    def get_further_angle(self, lidar):
        EVERY_N = 1

        lidar_dist = lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = lidar.ray_angles[::EVERY_N].copy()

        EVERY_N = 1
        LIDAR_DIST_CLIP = 40.0
        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9

        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)
        indice = np.argmax(lidar_dist_empty_clip)
        print(lidar_dist_empty_clip[indice])

        return lidar_angles[indice]



class MyMapMapping(MapAbstract):

    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (1113, 750)

        self._rescue_center = RescueCenter(size=(210, 90))
        self._rescue_center_pos = ((440, 315), 0)

        self._number_drones = 1
        self._drones_pos = [((-50, 0), 0)]
        self._drones = []

        self._playground = ClosedPlayground(size=self._size_area)

        self._playground.add(self._rescue_center, self._rescue_center_pos)

        add_walls(self._playground)
        add_boxes(self._playground)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            self._playground.add(drone, self._drones_pos[i])


def main():
    the_map = MyMapMapping(drone_type=MyDroneMapping)

    gui = GuiSR(the_map=the_map,
                use_keyboard=False,
                use_mouse_measure=True
                )
    gui.run()


if __name__ == '__main__':
    main()
