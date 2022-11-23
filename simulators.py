
from typing import *
import dataclasses
import abc

import traci
import traci.constants

from .utils import *

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

        # Vehicle repository
        self.vehicles: Dict[str, Vehicle] = {}

        # Simulator modules
        self.position_manager = PositionManager(self)
        self.network = NetworkSimulator(self)
        self.perception = PerceptionSimulator(self)


    def generate_vehicle(self, vid: str):
        # Generate one vehicle with specified id.

        # TODO: move this to init.
        possible_vehicle_types = [(i.__name__, i) for i in [
            UnconnectedVehicle,
            NormalVehicle,
            PoTVehicle,
            SilenceAttacker,
            SpamAttacker,
            RandomReplayAttacker,
            #SignalJammingAttacker,
            SybilAttacker,
        ]]

        vtp = self.config['vehicle_type_probabilities']
        vtp = dict((possible_vehicle_types[k], v) for k, v in vtp.items())
        assert sum(vtp.values()) == 1

        vehicle_class = self.vehicle_random.choices(vtp.keys(), vtp.values())[0]

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

        from vid in self.traci.simulation.getArrivedIDList():
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

        # TODO: naive implementation here.
        self._position = {}

    def get_vehicles_in_range(self, position: Position) -> List[Vehicle]:
        raise NotImplementedError

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle.numberplate]

    def update_all_position(self, tick: float):
        # Set vehicles' positions to the data at the given tick.

        for vid in self.vehicles.keys():
            x, y = self.traci.vehicle.getPosition(vid)
            yaw  = self.traci.vehicle.getAngle(vid)
            self._position[vid] = Position(x, y, yaw)


class NetworkSimulator(object):
    # Network simulator.
    # Currently a hand-crafted (dummy) implementation is used.

    def __init__(self, secenario: Scenario):
        self.secenario = scenario
        self.vehicles = scenario.vehicles
        self.random = random_from(scenario.random)
        self.pdr = scenario.config['v2v_pdr']

        self.position_manager = scenario.position_manager

        # Assign a receive buffer for each vehicle
        self.receive_buffers = dict((i, []) for i in self.vehicles)

    def broadcast(self, sender: Vehicle, message: object) -> int:

        receivers = position_manager.get_vehicles_in_range(sender.position)
        for i in receivers:
            if self.random.random() < self.pdr:
                self.receive_buffers[i].append(message)

        return len(receivers)

    def receive(self, receiver: Vehicle) -> list:
        ret = self.receive_buffers[receiver]
        self.receive_buffers[receiver] = []

        return ret

class PerceptionSimulator(object):
    def __init__(self, secenario: Scenario):
        self.secenario = scenario
        self.vehicles = scenario.vehicles
        self.random = random_from(scenario.random)

        self.position_manager = scenario.position_manager

        raise NotImplementedError


    def perceive(self, vehicle: Vehicle) -> List[Vehicle]:
        # Return all perceived vehicles of the given vehicle.
        raise NotImplementedError


