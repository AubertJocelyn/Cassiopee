from swarm_rescue.simulation.drone.controller import CommandsDict


class AimWounded:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isDroneAimingWounded():
            return self.drone.buildState("GraspWounded")
        else:
            return self

    def get_command(self):
        command: CommandsDict = {"forward": 0.0,
                                 "lateral": 0.0,
                                 "rotation": 1.0,
                                 "grasper": 0}
        return command