"""
This program can be launched directly.
"""

import math
import pathlib
import sys
from typing import List, Type, Tuple

import arcade
import numpy as np

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.utils.path import Path
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.utils.utils import clamp
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


class MyDronePidTranslation(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.counter = 0
        self.distance = 150.0
        self.position_setpoint = np.array([-self.distance, 0.0])
        self.counter_change_setpoint = 180

        self.iter_path = 0
        self.path_done = Path()
        self.prev_diff_position = 0

        self.to_the_right = True

        self.vitesse = 3.0
        self.arret = False
        self.compteur = 0
        self.distance = 0
        self.distance_arret = np.asarray([10, 10, 14, 14, 17, 25, 30, 37, 40, 50, 60, 70])

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self) -> CommandsDict:
        """
        The Drone will move a fix distance
        """
        command: CommandsDict = {"forward": 1.0,
                                 "rotation": 0.0}

        if self.arret:
            self.distance += self.measured_velocity()[0]

        if self.measured_velocity()[0] >= self.vitesse:
            self.arret = True

        if self.arret:
            self.compteur += 1
            command["forward"] = -1.0

        if self.measured_velocity()[0] < 0 and self.arret:
            print("distance:", self.distance)
            print("compteur", self.compteur)
            sys.exit(0)

        return command


class MyMap(MapAbstract):
    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (1000, 200)

        # POSITIONS OF THE DRONES
        self._number_drones = 1
        self._drones_pos = []
        pos = ((-400, 0), 0)
        self._drones_pos.append(pos)

        self._drones: List[DroneAbstract] = []

        self._playground = ClosedPlayground(size=self._size_area)

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
    the_map = MyMap(drone_type=MyDronePidTranslation)

    gui = GuiSR(the_map=the_map,
                use_keyboard=False,
                use_mouse_measure=True,
                enable_visu_noises=False,
                )

    gui.run()

    score_health_returned = the_map.compute_score_health_returned()
    print("score_health_returned = ", score_health_returned)


if __name__ == '__main__':
    main()