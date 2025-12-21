class GoToCenter:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isCenterNearby() and self.drone.getIsCarryingWounded():
            return self.drone.buildState("AimCenter")
        elif not (self.drone.isMapExplored() or self.drone.getIsCarryingWounded()):
            return self.drone.buildState("Explore")
        else:
            return self

    def get_command(self):
        self.drone.setRescueCenterPosition()
        self.drone.set_aimed_point_world_rescue_center()
        command = self.drone.getCommandForceCalculator()
        command["grasper"] = 1
        return command