from swarm_rescue.simulation.drone.controller import CommandsDict


class CheckIsCarryingWounded:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isWoundedVisible():
            self.drone.setCarryingWoundedTo(True)
            self.drone.resetPutWoundedInCenterTimer()
            return self.drone.buildState("PutWoundedInCenter")
        else:
            self.drone.setCarryingWoundedTo(False)
            self.drone.resetWoundedPersonPoint()
            return self.drone.buildState("Explore")

    def get_command(self):
        command: CommandsDict = {"forward": 0.0,
                                 "lateral": 0.0,
                                 "rotation": 0.0,
                                 "grasper": 0}
        return command