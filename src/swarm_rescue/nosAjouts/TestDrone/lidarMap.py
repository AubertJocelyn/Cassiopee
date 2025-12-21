"""
This program can be launched directly.
To move the drone, you have to click on the map, then use the arrows on the
keyboard
"""

import pathlib
import sys
from typing import Type

from swarm_rescue.nosAjouts.Submission.OurDrone import OurDrone

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.maps.walls_medium_02 import add_walls, add_boxes
from swarm_rescue.simulation.utils.misc_data import MiscData
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract

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
    the_map = MyMapMapping(drone_type=OurDrone)

    gui = GuiSR(the_map=the_map,
                use_keyboard=False,
                use_mouse_measure=True
                )
    gui.run()


if __name__ == '__main__':
    main()
