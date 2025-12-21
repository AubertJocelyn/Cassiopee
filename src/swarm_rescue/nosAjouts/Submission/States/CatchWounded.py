from swarm_rescue.simulation.drone.controller import CommandsDict
class CatchWounded:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if not self.drone.isWoundedVisible():
            self.drone.setCarryingWoundedTo(True)
        if self.drone.getIsCarryingWounded() and not self.drone.isCenterNearby():
            return self.drone.buildState("GoToCenter")
        elif self.drone.isWoundedNearby() and not self.drone.isDroneAimingWounded():
            return self.drone.buildState("AimWounded")
        else:
            return self

    def get_command(self):
        command: CommandsDict = {"forward": 1.0,
                                 "lateral": 0.0,
                                 "rotation": 0.0,
                                 "grasper": 1}
        return command