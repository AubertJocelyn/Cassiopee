
import pathlib
import sys
from typing import Type

import numpy as np

from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.maps.walls_medium_02 import add_walls, add_boxes
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


class MyDroneLidar(DroneAbstract):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.aim = 0

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self) -> CommandsDict:
        """
        We only send a command to do nothing
        """
        command: CommandsDict = {"forward": 0.2*np.cos(self.lidar().ray_angles[self.aim]),
                                 "lateral": 0.2*np.sin(self.lidar().ray_angles[self.aim]),
                                 "rotation": 0.0,
                                 "grasper": 0}
        if self.get_distance_aim(self.lidar()) < 150.0:
            command["forward"] = -0.2*np.cos(self.lidar().ray_angles[self.aim])
            command["lateral"] = -0.2*np.sin(self.lidar().ray_angles[self.aim])
            self.aim = (self.aim + 15) % 180
        return command

    def get_distance_aim(self, lidar):
        EVERY_N = 1
        LIDAR_DIST_CLIP = 40.0

        lidar_dist = lidar.get_sensor_values()[::EVERY_N].copy()
        lidar_angles = lidar.ray_angles[::EVERY_N].copy()


        max_range = MAX_RANGE_LIDAR_SENSOR * 0.9


        lidar_dist_empty = np.maximum(lidar_dist - LIDAR_DIST_CLIP, 0.0)
        lidar_dist_empty_clip = np.minimum(lidar_dist_empty, max_range)

        indice = np.where(lidar_angles == lidar_angles[self.aim])[0]

        return lidar_dist_empty_clip[indice]



class MyMapLidar(MapAbstract):

    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (1113, 750)

        self._rescue_center = RescueCenter(size=(210, 90))
        self._rescue_center_pos = ((440, 315), 0)

        self._number_drones = 1
        self._drones_pos = [((-50, 0), 3.1415 / 2)]
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
            drone = drone_type(identifier=i, misc_data=misc_data,
                               display_lidar_graph=True)
            self._drones.append(drone)
            self._playground.add(drone, self._drones_pos[i])


def main():
    the_map = MyMapLidar(drone_type=MyDroneLidar)

    # draw_lidar_rays : enable the visualization of the lidar rays
    # enable_visu_noises : to enable the visualization. It will show also a
    # demonstration of the integration of odometer values, by drawing the
    # estimated path in red. The green circle shows the position of drone
    # according to the gps sensor and the compass
    gui = GuiSR(the_map=the_map)
    gui.run()


if __name__ == '__main__':
    main()