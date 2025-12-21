import pathlib
import random
import sys
from enum import Enum
from typing import Optional, List, Type

import numpy as np

from swarm_rescue.nosAjouts.Submission.Rescue.WoundedRescueController import WoundedRescueController
from swarm_rescue.nosAjouts.Submission.States.AimWounded import AimWounded

# Insert the 'src' directory, located two levels up from the current script,
# into sys.path. This ensures Python can find project-specific modules
# (e.g., 'swarm_rescue') when the script is run from a subfolder like 'examples/'.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.elements.wounded_person import WoundedPerson
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData


class MyDroneSemantic(DroneAbstract):
    class Activity(Enum):
        """
        All the states of the drone as a state machine
        """
        SEARCHING_WOUNDED = 1
        GRASPING_WOUNDED = 2
        SEARCHING_RESCUE_CENTER = 3
        DROPPING_AT_RESCUE_CENTER = 4

    def __init__(self,
                 identifier: Optional[int] = None, **kwargs):
        super().__init__(identifier=identifier,
                         display_lidar_graph=False,
                         **kwargs)
        # The state is initialized to searching wounded person
        self.state = self.Activity.SEARCHING_WOUNDED

        # Those values are used by the random control function
        self.counterStraight = 0
        self.angleStopTurning = 0
        self.isTurning = False
        self.counter = 0
        self.woundedRescueController = WoundedRescueController()
        self.state = "Research"
        self.decompte = 100
        self.angle_ref = 0
        self.state = AimWounded(self)

    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass

    def control(self) -> CommandsDict:
        self.woundedRescueController.update(self.semantic_values())
        self.state = self.state.update_state()
        return self.state.get_command()

    def isDroneAimingRescueCenter(self):
        return self.woundedRescueController.isDroneAimingCenter()

    def isDroneAimingWounded(self):
        return self.woundedRescueController.isDroneAimingWounded()

    def isWoundedVisible(self):
        return self.woundedRescueController.isWoundedVisible()

    def getCorrectionRotation(self):
        return self.woundedRescueController.getCorrectionRotation(self.measured_compass_angle())

    def setRescueCenterAngle(self):
        self.woundedRescueController.setRescueCenterAngle(self.measured_compass_angle())



class MyMapSemantic(MapAbstract):
    def __init__(self, drone_type: Type[DroneAbstract]):
        super().__init__(drone_type=drone_type)

        # PARAMETERS MAP
        self._size_area = (400, 400)

        self._rescue_center = RescueCenter(size=(100, 100))
        self._rescue_center_pos = ((0, 150), 0)

        # WOUNDED PERSONS
        self._number_wounded_persons = 1
        self._wounded_persons_pos = []
        self._wounded_persons: List[WoundedPerson] = []

        pos = ((30, 0), 0.0)
        self._wounded_persons_pos.append(pos)

        # POSITIONS OF THE DRONES
        self._number_drones = 1
        self._drones_pos = [((0, 0), np.pi*random.randint(-179, 179)/180)]
        self._drones = []

        self._playground = ClosedPlayground(size=self._size_area)

        self._playground.add(self._rescue_center, self._rescue_center_pos)

        # POSITIONS OF THE WOUNDED PERSONS
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = self._wounded_persons_pos[i]
            self._playground.add(wounded_person, pos)

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
    the_map = MyMapSemantic(drone_type=MyDroneSemantic)

    # draw_semantic_rays : enable the visualization of the semantic rays
    gui = GuiSR(the_map=the_map,
                draw_semantic_rays=True,
                use_keyboard=False,
                )
    gui.run()


if __name__ == '__main__':
    main()
