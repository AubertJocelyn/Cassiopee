class Explore:
    def __init__(self, drone):
        self.drone = drone

    def get_next_state(self):
        if self.drone.isWoundedDiscovered() and self.drone.isCenterDiscovered():
            return self.drone.buildState("GoToWounded")
        elif self.drone.isMapExplored():
            return self.drone.buildState("GoToCenter")
        else:
            return self


    def get_command(self):
        self.drone.set_aimed_point_world_unexplored()
        command = self.drone.getCommandForceCalculator()
        command["grasper"] = 0
        return command
