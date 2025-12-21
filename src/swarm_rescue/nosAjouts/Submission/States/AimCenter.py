from swarm_rescue.simulation.drone.controller import CommandsDict


class AimCenter:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isDroneAimingCenter():
            self.drone.setRescueCenterAngle()
            return self.drone.buildState("PutWoundedInCenter")
        elif not (self.drone.isCenterNearby() and self.drone.isCenterVisible()):
            return self.drone.buildState("GoToCenter")
        else:
            return self

    def get_command(self):
        command: CommandsDict = {"forward": 0.0,
                                 "lateral": 0.0,
                                 "rotation": 1.0,
                                 "grasper": 1}
        return command

