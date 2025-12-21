from swarm_rescue.simulation.drone.controller import CommandsDict


class PutWoundedInCenter:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.getPutWoundedInCenterTimer() == 0:
            return self.drone.buildState("CheckIsCarryingWounded")
        else:
            self.drone.decreasePutWoundedInCenterTimer()
            return self

    def get_command(self):
        command: CommandsDict = {"forward": 1.0,
                                 "lateral": 0.0,
                                 "rotation": 0.0,
                                 "grasper": 1}
        command["rotation"] = self.drone.getCorrectionRotation()
        return command