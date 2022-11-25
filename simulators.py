
from typing import *
import dataclasses
import abc

import traci
import traci.constants

from vehicle import *
from utils import *

class Scenario(object):
    # The top-level simulation scenario.
    def __init__(self, config):
        self.config = config
        self.traci = traci

        self.seed = self.config.get('seed', random.randint(0, 2 ** 32 - 1))
        # Master random, only used for deriving other randoms.
        self.random = random_from(self.seed)
        # Vehicle random, used to select vehicle type, and derive randoms for vehicle instances.
        self.vehicle_random = random_from(self.random)

        # Vehicle repository.
        self.vehicles: Dict[str, Vehicle] = {}

        # Simulator modules.
        self.position_manager = PositionManager(self)
        self.network = NetworkSimulator(self)
        self.perception = PerceptionSimulator(self)

        # Initialize vehicle type probability from config file.
        possible_vehicle_types = [(i.__name__, i) for i in [
            UnconnectedVehicle,
            ConnectedVehicle,
            PoTVehicle,
            SilenceAttacker,
            SpamAttacker,
            RandomReplayAttacker,
            #SignalJammingAttacker,
            SybilAttacker,
        ]]

        self.vtp = self.config['vehicle_type_probabilities']
        self.vtp = dict((possible_vehicle_types[k], v) for k, v in self.vtp.items())
        assert sum(self.vtp.values()) == 1

    def generate_vehicle(self, vid: str):
        # Generate one vehicle with specified id.
        vehicle_class = self.vehicle_random.choices(self.vtp.keys(), self.vtp.values())[0]

        ret = vehicle_class(
            scenario = self,
            numberplate = vid,
            config = config['vehicle'],
            trace = self.sumo.vehicle_traces[vid],
            random = random_from(self.vehicle_random),
        )

        return ret

    def tick(self, tick):
        # Run one traci step first.
        traci.simulationStep()

        # Update vehicle list from traci.
        for vid in self.traci.simulation.getDepartedIDList():
            if vid not in self.vehicles:
                self.vehicles[vid] = self.generate_vehicle(vid)

        for vid in self.traci.simulation.getArrivedIDList():
            self.vehicles[vid].stop()
            del self.vehicles[vid]

        # Let position manager to update vehicles' position.
        self.position_manager.update_all_position(tick)

        # Do a tick for every running vehicles.
        for v in self.vehicles:
            v.tick()


    def collect_metrics(self):
        raise NotImplementedError



class PositionManager(object):
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles
        self.traci = scenario.traci
        self.config = scenario.config['position_manager']
        self.range_limit = self.config['range_limit']

        # TODO: naive implementation here.
        # TODO: thread safety.
        self._position: Dict[Vehicle, Position] = {}

    def get_vehicles_in_range(self, position: Position) -> List[Vehicle]:
        # TODO: O(N) implementation.
        ret = []

        for v, pos in self._position.items():
            if (pos - position).to_polar()[0] < self.range_limit:
                ret.append(v)

        return ret

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle.numberplate]

    def update_all_position(self, tick: float):
        # Set vehicles' positions to the data at the given tick.
        self._position = {}

        for v in self.vehicles.keys():
            x, y = self.traci.vehicle.getPosition(v.numberplate)
            yaw  = self.traci.vehicle.getAngle(v.numberplate)
            self._position[v] = Position(x, y, yaw)


class NetworkSimulator(object):
    # Network simulator.
    # Currently a hand-crafted (dummy) implementation is used.

    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles
        self.random = random_from(scenario.random)
        self.config = scenario.config['network_simulator']
        self.pdr = self.config['v2v_pdr']

        self.position_manager = scenario.position_manager

        # Assign a receive buffer for each vehicle
        self.receive_buffers = {}

    def broadcast(self, sender: Vehicle, message: object) -> int:
        # Broadcast a message to vehicles in range and return number of receipents.
        receivers = position_manager.get_vehicles_in_range(sender.position)
        for v in receivers:
            if v not in self.receive_buffers:
                self.receive_buffers[v] = []

            if self.random.random() < self.pdr:
                self.receive_buffers[v].append(message)

        return len(receivers)

    def receive(self, receiver: Vehicle) -> list:
        # Pop a vehicle's receive buffer.
        ret = self.receive_buffers.get(receiver, [])
        self.receive_buffers[receiver] = []

        return ret

class PerceptionSimulator(object):
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles
        self.random = random_from(scenario.random)
        self.config = scenario.config['perception_simulator']
        self.vision_distance = self.config['vision_distance']
        self.vision_angle = abs(self.config['vision_angle'])

        self.position_manager = scenario.position_manager


    def perceive(self, vehicle: Vehicle) -> List[Vehicle]:
        # Return all perceived vehicles of the given vehicle.
        candidates = position_manager.get_vehicles_in_range(self.position)

        ret = []

        for v in candidates:
            delta = v.position - self.position
            distance, angle = delta.to_polar()
            if distance < self.vision_distance and abs(angle) < self.vision_angle:
                ret.append(v)

        return v
