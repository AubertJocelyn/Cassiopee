import numpy as np

from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor
MIN_DISTANCE_CENTER = 200

class RescueAnalyzer:
    def __init__(self):

        self.x_wounded = 0.0
        self.y_wounded = 0.0

        self.x_center = 0.0
        self.y_center = 0.0
        self.global_min_dist = MIN_DISTANCE_CENTER
        self.max_number_of_points = 0
        self.local_min_dist = MIN_DISTANCE_CENTER
        self.number_of_points = 0

        self.wounded_datas = []
        self.rescue_datas = []

    def update(self, drone_pos, semantic_values):
        self.local_min_dist = MIN_DISTANCE_CENTER
        self.number_of_points = 0

        self.rescue_datas = []
        self.wounded_datas = []
        for data in semantic_values:
            if data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON:
                self.wounded_datas.append(data)
            elif data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER:
                self.rescue_datas.append(data)

        self.AnalyzeRescue(drone_pos.position[0], drone_pos.position[1])
        self.AnalyzeWounded(drone_pos)

    def AnalyzeWounded(self, drone_pos):
        if len(self.wounded_datas) > 0:
            center_indice = len(self.wounded_datas) // 2
            center_wounded_data = self.wounded_datas[center_indice]
            self.x_wounded = drone_pos.position[0] + center_wounded_data.distance * np.cos(
                center_wounded_data.angle + drone_pos.orientation)
            self.y_wounded = drone_pos.position[1] + center_wounded_data.distance * np.sin(
                center_wounded_data.angle + drone_pos.orientation)

    def AnalyzeRescue(self, x_world, y_world):
        self.number_of_points = len(self.rescue_datas)
        if self.number_of_points > 0:
            self.local_min_dist = min([min(self.local_min_dist, data.distance) for data in self.rescue_datas])
        MIN_DISTANCE = 50
        if MIN_DISTANCE < self.local_min_dist:
            if self.number_of_points > self.max_number_of_points \
                    or (
                    self.number_of_points == self.max_number_of_points and self.local_min_dist < self.global_min_dist):
                self.global_min_dist = self.local_min_dist
                self.x_center = x_world
                self.y_center = y_world
                self.max_number_of_points = self.number_of_points

    def isMeanZero(self, list):
        if len(list) > 1:
            if abs(sum(list) / len(list)) < np.pi / 32:
                if max([abs(e) for e in list]) < np.pi / 2:
                    return True
        return False