
from typing import *
import dataclasses
import atexit
import abc

import traci
import math

from vehicle import *
from modules import *
from utils import *

class Scenario(object):
    # The top-level simulation scenario.
    def __init__(self, config):
        self.traci = traci
        self.config = config
        self.use_gui = self.config['use_gui']
        
        self.init_sumo()
        self.init_vtp()
        self.init_rng()

        # Create vehicle repository.
        self.vehicles: Dict[str, Vehicle] = {}

        # Create simulator modules.
        self.position_manager = PositionManager(self)
        #self.position_manager = PositionManagerV2(self)
        self.network = NetworkSimulator(self)
        self.perception = PerceptionSimulator(self)
        self.match = MatchSimulator(self)

        atexit.register(self.cleanup)

    def init_vtp(self):
        # Initialize vehicle type probability from config file.
        possible_vehicle_types = dict([(i.__name__, i) for i in [
            UnconnectedVehicle,
            ConnectedVehicle,
            PoTVehicle,
            SilenceAttacker,
            SpamAttacker,
            ReplayAttacker,
            #SignalJammingAttacker,
            SybilAttacker,
        ]])

        vtp = self.config['vehicle_type_probabilities']
        self.vtp = dict((possible_vehicle_types[k], v) for k, v in vtp.items())
        assert sum(self.vtp.values()) == 1

    def init_rng(self):
        self.seed = self.config.get('seed', random.randint(0, 2 ** 32 - 1))
        # Master random, only used for deriving other randoms.
        self.random = random_from(self.seed)
        # Vehicle random, used to select vehicle type, and derive randoms for vehicle instances.
        self.vehicle_random = random_from(self.random)

    def init_sumo(self):
        sumo_bin = 'sumo-gui' if self.use_gui else 'sumo'

        self.traci.start([sumo_bin, "-c", self.config['sumo_config_path']])

        self.now = self.traci.simulation.getTime()
        self.end_time = self.traci.simulation.getEndTime()

    def cleanup(self):
        self.traci.close()

        atexit.unregister(self.cleanup)

    def generate_vehicle(self, vid: str):
        # Generate one vehicle with specified id.
        vehicle_class = self.vehicle_random.choices(list(self.vtp.keys()), list(self.vtp.values()))[0]

        ret = vehicle_class(
            scenario = self,
            numberplate = vid,
            config = self.config['vehicle'],
            random = random_from(self.vehicle_random),
        )

        return ret

    def tick(self):
        # Run one traci step first.
        traci.simulationStep()

        self.now = traci.simulation.getTime()
        if self.now >= self.end_time:
            raise StopIteration
        print("Simulation time: %.2f" % self.now)

        # Update vehicle list from traci.
        for vid in self.traci.simulation.getDepartedIDList():
            if vid not in self.vehicles:
                self.vehicles[vid] = self.generate_vehicle(vid)

                # Set default color.
                self.set_vehicle_color(vid, (128, 128, 128, 255))

        for vid in self.traci.simulation.getArrivedIDList():
            self.vehicles[vid].stop()
            del self.vehicles[vid]

        # Give vehicles chance to thange their EIDs and let match simulator know.
        [ v.possibly_change_eid() for v in self.vehicles.values()]
        self.match.update_eids()

        # Let position manager to update vehicles' position and update to all vehicles.
        self.position_manager.update_all_position()
        [v.update_position() for v in self.vehicles.values()]
        
        # Do a tick for every running vehicles.
        for v in self.vehicles.values():
            v.do_work()

    def collect_metrics(self):
        # Collect metrics from all vehicles.

        recent_saw_by: VehicleMetric = self.collect_recent_saw_by()

        if not self.use_gui:
            print('recent_saw_by = %s' % recent_saw_by)

        def _normalize_color(n, max_ = 10, min_ = 0) -> int:
            # Normalize and clip a number to 0-255.
            ret = int(255. * (n - min_) / (max_ - min_))
            return min(max(ret, 0), 255)

        for vid in self.vehicles.keys():
            # Set color according to metrics.

            r = _normalize_color(recent_saw_by[vid], max_=10)
            g = 0
            b = 0

            self.set_vehicle_color(vid, (r, g, b, 255))

    def set_vehicle_color(self, vid: str, color: Tuple[int, int, int, int]):
        # Set vehicle color if gui is enabled.
        if self.use_gui:
            self.traci.vehicle.setColor(vid, color)

    # Metric collectors.

    def collect_recent_saw_by(self) -> VehicleMetric:
        # Collect how many vehicles saw a given vehicle in the last 10 seconds.

        ret = dict((id, 0) for id in self.vehicles.keys())

        for v0 in self.vehicles.values():
            module = v0.get_module(Planner)
            if not module:
                continue
            for v1 in module.recent_seen_vehicles():
                ret[v1.numberplate] += 1

        return ret

    def collect_prover_matches(self) -> VehicleMetric:
        # Collect how many match entries of a given prover.

        ret = dict((id, 0) for id in self.vehicles.keys())

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            ret[v.numberplate] = module and len(module._numberplate_to_eid)

        return ret

    def collect_prover_unmatched_eids(self) -> VehicleMetric:
        # Collect how many unmatched eids of a given prover.

        ret = dict((id, 0) for id in self.vehicles.keys())

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            ret[v.numberplate] = module and len(module.unmatched_eids)

        return ret

    def collect_prover_known_numberplates(self) -> VehicleMetric:
        # Collect how many known numberplates of a given prover.

        ret = dict((id, 0) for id in self.vehicles.keys())

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            ret[v.numberplate] = module and len(module.known_numberplates)

        return ret


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

    def get_nearby_vehicles(self, position: Position) -> List[Vehicle]:
        # TODO: O(N) implementation.
        ret = []

        for v, pos in self._position.items():
            if (pos - position).to_polar()[0] < self.range_limit:
                ret.append(v)

        return ret

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle]

    def update_all_position(self):
        # Set vehicles' positions to the data at the given tick.
        self._position = {}

        for numberplate, v in self.vehicles.items():
            x, y = self.traci.vehicle.getPosition(numberplate)
            yaw  = self.traci.vehicle.getAngle(numberplate)

            # Warning: getAngle returns yaw (0 for North, 90 for East, etc.)
            # Need to convert it to theta (0 for East, 90 for North, etc.)
            theta = (360 + 90 - yaw) % 360
            self._position[v] = Position(x, y, theta)

class PositionManagerV2(object):
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles
        self.traci = scenario.traci
        self.config = scenario.config['position_manager']
        self.range_limit = self.config['range_limit']

        grid_size = self.range_limit

        self.boundary = self.traci.simulation.getNetBoundary()
        assert self.boundary[0] == (0, 0)

        # Create a grid of grid_x and grid_y with padding.
        grid_x = math.ceil(self.boundary[1][0] / self.range_limit)
        grid_y = math.ceil(self.boundary[1][1] / self.range_limit)

        self._grid = [[set() for _ in range(grid_y + 2)] for _ in range(grid_x + 2)]

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle]

    def get_nearby_vehicles(self, position: Position) -> List[Vehicle]:
        # Get nearby vehicles from given position.

        # Offset by one since the grid is padded.
        grid_x = math.floor(position.x / self.range_limit) + 1
        grid_y = math.floor(position.y / self.range_limit) + 1

        # Get all vehicles in the 9 adjacent grids as candidates.
        candidates = set.union(*[self._grid[grid_x + i][grid_y + j] for i in [-1, 0, 1] for j in [-1, 0, 1]])

        # Filter out vehicles > range_limit away.
        ret = [ v for v in candidates if (self._position[v] - position).to_polar()[0] < self.range_limit ]

        return ret

    def update_all_position(self):
        # Set vehicles' positions to the data at the given tick.
        self._position = {}

        for numberplate, v in self.vehicles.items():
            x, y = self.traci.vehicle.getPosition(numberplate)
            yaw  = self.traci.vehicle.getAngle(numberplate)

            # Warning: getAngle returns yaw (0 for North, 90 for East, etc.)
            # Need to convert it to theta (0 for East, 90 for North, etc.)
            theta = (360 + 90 - yaw) % 360
            self._position[v] = Position(x, y, theta)

        # Recreate grid.
        self._grid = [[set() for _ in range(len(self._grid[0]))] for _ in range(len(self._grid))]

        for v, pos in self._position.items():
            # Offset by one since the grid is padded.
            grid_x = math.floor(pos.x / self.range_limit) + 1
            grid_y = math.floor(pos.y / self.range_limit) + 1

            self._grid[grid_x][grid_y].add(v)

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
        receivers = self.position_manager.get_nearby_vehicles(sender.position)
        for v in receivers:
            if v == sender: continue # Don't send to self.

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


    def perceive(self, ego: Vehicle) -> List[Vehicle]:
        # Return all perceived vehicles of the given egovehicle.
        candidates = self.position_manager.get_nearby_vehicles(ego.position)

        ret = []

        for v in candidates:
            if v == ego: continue # Don't count self.

            distance, angle = (v.position - ego.position).to_polar()
            # Do not use > here since it may be nan.
            if (
                distance < self.vision_distance and 
                abs(ego.position.heading - angle) < self.vision_angle
            ):
                ret.append(v)

        return ret

class MatchSimulator(object):
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles

        self.eid_to_numberplate: Dict[EID, NumberPlate] = {}

    def update_eids(self):
        # Update EIDs of all vehicles.
        # Only keep the latest EID of each vehicle.

        self.eid_to_numberplate = dict(
            (v.eid, v.numberplate) for v in self.vehicles.values()
        )

    def match_eid(self, known_numberplates: Set[NumberPlate], eid: EID) -> NumberPlate:
        # Match a EID to a numberplate in candidates.
        # Return the number plate if found, otherwise return None.
        if self.eid_to_numberplate.get(eid, None) in known_numberplates:
            return self.eid_to_numberplate[eid]
        return None

