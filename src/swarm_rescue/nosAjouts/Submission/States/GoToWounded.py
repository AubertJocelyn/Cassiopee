class GoToWounded:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isWoundedVisible() and self.drone.isWoundedNearby():
            return self.drone.buildState("AimWounded")
        else:
            return self

    def get_command(self):
        self.drone.setWoundedPosition()
        self.drone.set_aimed_point_world_wounded()
        return self.drone.getCommandForceCalculator()