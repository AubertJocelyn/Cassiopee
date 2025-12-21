from abc import abstractmethod

from swarm_rescue.nosAjouts.Submission.Rescue.RescueAnalyzer import RescueAnalyzer
from swarm_rescue.simulation.utils.pose import Pose


class RescueAbilitiesMixin:
    rescueAnalyzer: RescueAnalyzer
    x_drone: float
    y_drone: float
    @abstractmethod
    def getCurrentAngle(self):
        pass

    def isCenterDiscovered(self):
        return self.rescueAnalyzer.x_center != self.rescueAnalyzer.y_center != 0

    def isWoundedDiscovered(self):
        return self.rescueAnalyzer.x_wounded != self.rescueAnalyzer.y_wounded != 0.0

    def isCenterVisible(self):
        return len(self.rescueAnalyzer.rescue_datas) > 0

    def isWoundedVisible(self):
        return len(self.rescueAnalyzer.wounded_datas) > 0

    def isDroneAimingCenter(self):
        return self.rescueAnalyzer.isMeanZero([data.angle for data in self.rescueAnalyzer.rescue_datas])

    def isDroneAimingWounded(self):
        return self.rescueAnalyzer.isMeanZero([data.angle for data in self.rescueAnalyzer.wounded_datas])

    def getIsCarryingWounded(self):
        return self.isCarryingWounded

    def setCarryingWoundedTo(self, bool):
        self.isCarryingWounded = bool

    def getCorrectionRotation(self):
        CORRECTION = 0.2
        if self.getCurrentAngle() < self.rescue_center_angle:
            return CORRECTION
        else:
            return - CORRECTION

    def setRescueCenterAngle(self):
        self.rescue_center_angle = self.getCurrentAngle()

    def resetWoundedPersonPoint(self):
        self.rescueAnalyzer.x_wounded = 0
        self.rescueAnalyzer.y_wounded = 0

    def isCenterNearby(self):
        if self.isCenterDiscovered():
            DISTANCE = 150
            x = (self.rescueAnalyzer.x_center - self.x_drone)**2
            y = (self.rescueAnalyzer.y_center - self.y_drone)**2
            return (x + y)**0.5 < DISTANCE
        return False

    def isWoundedNearby(self):
        if self.isWoundedDiscovered():
            DISTANCE = 80
            x = (self.rescueAnalyzer.x_wounded - self.x_drone) ** 2
            y = (self.rescueAnalyzer.y_wounded - self.y_drone) ** 2
            return (x + y) ** 0.5 < DISTANCE
        return False