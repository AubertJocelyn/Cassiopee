import numpy as np

from swarm_rescue.nosAjouts.Submission.Cartographer import Cartographer
from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.maps.walls_medium_02 import add_walls, add_boxes
from swarm_rescue.simulation.utils.constants import MAX_RANGE_LIDAR_SENSOR
from swarm_rescue.simulation.utils.grid import Grid
from swarm_rescue.simulation.utils.misc_data import MiscData
from swarm_rescue.simulation.utils.pose import Pose
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.elements.rescue_center import RescueCenter
from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
class OurDrone(DroneAbstract):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.Cartographer = Cartographer(resolution=8, self_area_world=self.size_area)

    def define_message_for_all(self):
        pass

    def control(self) -> CommandsDict:
        robot_pose = Pose(np.asarray(self.measured_gps_position()),
                                   self.measured_compass_angle())
        lidar_values = {
            "lidar_angles": self.lidar().ray_angles.copy(),
            "lidar_dist": self.lidar().get_sensor_values().copy()
        }

        self.Cartographer.update_map(robot_pose=robot_pose, lidar_values=lidar_values)

        command: CommandsDict = {"forward": 0.0,
                                 "lateral": 0.0,
                                 "rotation": 0.0,
                                 "grasper": 0}
        return command