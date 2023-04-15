
from typing import *
import dataclasses
import atexit
import abc
import numpy as np
import math

import traci
import math
import time

from vehicle import *
from modules import *
from utils import *

class Scenario(object):
    # The top-level simulation scenario.
    def __init__(self, config):
        self.traci = traci
        self.config = config
        self.use_gui = self.config['use_gui']
        self.output_dir = self.config['output_dir']
        
        self.init_sumo()
        self.init_vtp()
        self.init_rng()

        # Create vehicle repository.
        self.vehicles: Dict[str, Vehicle] = {}

        # Create simulator modules.
        #self.position_manager = PositionManager(self)
        self.position_manager = PositionManagerV2(self)
        self.network = NetworkSimulator(self)
        #self.perception = PerceptionSimulator(self)
        self.perception = PerceptionSimulatorV3(self)
        self.match = MatchSimulator(self)

        # Create metric collectors.
        os.makedirs(self.output_dir, exist_ok=True)
        format_dir = lambda x: os.path.join(self.output_dir, x)

        self.metric_collectors = {
            'latitude': MetricCollector(format_dir('latitude.json'), lambda: {
                vid: v.position.x for vid, v in self.vehicles.items()
            }, metric_type = 'vehicle'),
            'longitude': MetricCollector(format_dir('longitude.json'), lambda: {
                vid: v.position.y for vid, v in self.vehicles.items()
            }, metric_type = 'vehicle'),
            'recent_saw_by': MetricCollector(format_dir('recent_saw_by.json'), self.collect_recent_saw_by),
            'bytes_sent': MetricCollector(format_dir('bytes_sent.json'), self.collect_vehicle_sent_bytes),
            'enqueued_proofs': MetricCollector(format_dir('enqueued_proofs.json'), self.collect_enqueued_proofs),
            'dropped_proofs': MetricCollector(format_dir('dropped_proofs.json'), self.collect_dropped_proofs),
        }

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

        print("Initialized sumo with end time: %s" % self.end_time)

    def cleanup(self):
        self.collect_final_metrics()

        for mc in self.metric_collectors.values():
            mc.save()

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

        t0 = time.time()

        # Run one traci step first.
        self.traci.simulationStep()

        self.now = traci.simulation.getTime()
        if self.now >= self.end_time:
            raise StopIteration

        # Update vehicle list from traci.
        for vid in self.traci.simulation.getDepartedIDList():
            if vid not in self.vehicles:
                self.vehicles[vid] = self.generate_vehicle(vid)
                #print("New %s: %s" % (self.vehicles[vid].__class__.__name__, vid))

        for vid in self.traci.simulation.getArrivedIDList():
            #self.vehicles[vid].stop()
            del self.vehicles[vid]

        # Give vehicles chance to thange their EIDs and let match simulator know.
        [ v.possibly_change_eid() for v in self.vehicles.values()]
        self.match.update_eids()

        # Let position manager to update vehicles' position and update to all vehicles.
        self.position_manager.update_all_position()
        [v.update_position() for v in self.vehicles.values()]

        t3 = time.time()
        
        # Do a tick for every running vehicles.
        for v in self.vehicles.values():
            v.do_work()

        t4 = time.time()

        # Collect metrics from all vehicles.
        [i.collect() for i in self.metric_collectors.values()]

        t5 = time.time()

        # Print out some statistics.
        format_time = lambda dt: "%3.0f ms(%3.0f us)" % (dt * 1000, dt * 1000 * 1000 / len(self.vehicles))

        print("step: %s; work: %s; coll: %s; total: %s; tick: %.0f; #vehicles: %d" % (
            format_time(t3 - t0),
            format_time(t4 - t3),
            format_time(t5 - t4),
            format_time(t5 - t0),
            self.now,
            len(self.vehicles),
        ))

    def collect_final_metrics(self):
        # Collect final metrics from all vehicles.
        pass

    def set_vehicle_color(self, vid: str, color: Tuple[int, int, int, int]):
        # Set vehicle color if gui is enabled.
        if self.use_gui:
            self.traci.vehicle.setColor(vid, color)

    # Metric collectors.

    def collect_vehicle_sent_bytes(self) -> VehicleMetric:
        # Collect how many bytes a vehicle sent in this tick.
        ret = dict((id, 0) for id in self.vehicles.keys())

        for v, b in self.network.bytes_sent.items():
            ret[v.numberplate] += b

        #for v in self.vehicles.values():
        #    ret[v.numberplate] += self.network.bytes_sent[v]

        # XXX: reset bytes_sent here.
        self.network.bytes_sent = {}

        return ret

    def collect_recent_saw_by(self) -> VehicleMetric:
        # Collect how many vehicles saw a given vehicle in the last 10 seconds.

        ret = dict((id, 0) for id in self.vehicles.keys())
        ret[UNKNOWN_PLATE] = 0

        for v in self.vehicles.values():
            module = v.get_module(Planner)
            if not module:
                continue
            for objid in module.recent_seen_objids():
                try:
                    numberplate = self.perception._gt_objectid_to_numberplate[objid]
                    ret[numberplate] += 1
                except (IndexError, KeyError):
                    # IndexError is for gt_objectid_to_numberplate; KeyError is for ret.
                    ret[UNKNOWN_PLATE] += 1

        return ret

    def collect_enqueued_proofs(self) -> VehicleMetric:
        # Collect how many proofs a vehicle enqueued in this tick.
        ret = {}

        for v in self.vehicles.values():
            module = v.get_module(PoTProver)
            if not module:
                continue
            ret[v.numberplate] = module.n_enqueued_proofs

            # XXX: reset n_enqueued_proofs here.
            module.n_enqueued_proofs = 0

        return ret

    def collect_dropped_proofs(self) -> VehicleMetric:
        # Collect how many proofs a vehicle dropped in this tick.
        ret = {}

        for v in self.vehicles.values():
            module = v.get_module(PoTProver)
            if not module:
                continue
            ret[v.numberplate] = module.n_dropped_proofs

            # XXX: reset n_dropped_proofs here.
            module.n_dropped_proofs = 0

        return ret

    def collect_prover_matches(self) -> VehicleMetric:
        # Collect how many match entries of a given prover.

        ret = {}

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            if not module:
                continue
            ret[v.numberplate] = len(module._numberplate_to_eid)

        return ret

    def collect_prover_unmatched_eids(self) -> VehicleMetric:
        # Collect how many unmatched eids of a given prover.

        ret = {}

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            if not module:
                continue
            ret[v.numberplate] = len(module.unmatched_eids)

        return ret

    def collect_prover_known_numberplates(self) -> VehicleMetric:
        # Collect how many known numberplates of a given prover.

        ret = {}

        for v in self.vehicles.values():
            module = v.get_module(Prover)
            if not module:
                continue
            ret[v.numberplate] = len(module.known_numberplates)

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

        self.subscribed = set()

        grid_size = self.range_limit

        self.boundary = self.traci.simulation.getNetBoundary()
        assert self.boundary[0] == (0, 0)

        # Create a grid of grid_x and grid_y with padding.
        grid_x = math.ceil(self.boundary[1][0] / self.range_limit) + 2
        grid_y = math.ceil(self.boundary[1][1] / self.range_limit) + 2

        self._grid = [[set() for _ in range(grid_y)] for _ in range(grid_x)]

        # Cache results to speed up get_nearby_vehicles.
        self.result_cache = {}

        print("PositionManagerV2 initialized with boundary %s, grid size %d x %d" % (self.boundary, grid_x, grid_y))

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle]

    def get_nearby_vehicles(self, position: Position) -> List[Vehicle]:
        # Get nearby vehicles from given position.

        if position in self.result_cache:
            return self.result_cache[position]

        # Offset by one since the grid is padded.
        grid_x = math.floor(position.x / self.range_limit) + 1
        grid_y = math.floor(position.y / self.range_limit) + 1

        # Return nothing if the position is out of range.
        # Be careful about the padding.
        if not 0 < grid_x <= self.boundary[1][0] or not 0 < grid_y <= self.boundary[1][1]:
            return []

        # Get all vehicles in the 9 adjacent grids as candidates.
        candidates = set.union(*[self._grid[grid_x + i][grid_y + j] for i in [-1, 0, 1] for j in [-1, 0, 1]])

        # Filter out vehicles > range_limit away.
        ret = [ v for v in candidates if self._position[v].distance_to(position) < self.range_limit ]
        self.result_cache[position] = ret

        return ret

    def update_all_position(self):
        # Set vehicles' positions to the data at the given tick.
        self._position = {}

        # Invalidate cache.
        self.result_cache = {}

        for numberplate, v in self.vehicles.items():
            # Subscribe to the vehicle if not already subscribed.
            if numberplate not in self.subscribed:
                self.traci.vehicle.subscribe(numberplate, [traci.constants.VAR_POSITION, traci.constants.VAR_ANGLE])
                self.subscribed.add(numberplate)

        result = self.traci.vehicle.getAllSubscriptionResults()

        for numberplate, v in self.vehicles.items():
            #x, y = self.traci.vehicle.getPosition(numberplate)
            #yaw  = self.traci.vehicle.getAngle(numberplate)
            x, y = result[numberplate][traci.constants.VAR_POSITION]
            yaw  = result[numberplate][traci.constants.VAR_ANGLE]

            # Warning: getAngle returns yaw (0 for North, 90 for East, etc.)
            # Need to convert it to theta (0 for East, 90 for North, etc.)
            theta = (360 + 90 - yaw) % 360
            self._position[v] = Position(x, y, theta)

        # TODO: Do we need to unsubscribe vehicles that are no longer in the simulation?

        # Update the grid.
        self._update_grid()

    def _update_grid(self):
        # Recreate grid.
        self._grid = [[set() for _ in range(len(self._grid[0]))] for _ in range(len(self._grid))]

        for v, pos in self._position.items():
            # Offset by one since the grid is padded.
            grid_x = math.floor(pos.x / self.range_limit) + 1
            grid_y = math.floor(pos.y / self.range_limit) + 1

            # Do not count this vehicle if the position is out of range.
            # Be careful about the padding.
            if not 0 < grid_x <= self.boundary[1][0] or not 0 < grid_y <= self.boundary[1][1]:
                #print('WARNING: vehicle %s is out of range: %s, yaw: %.2f theta: %.2f' % (v.numberplate, pos, yaw, theta))
                continue

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

        self.bytes_sent = {}

    def broadcast(self, sender: Vehicle, message: object) -> int:
        # Broadcast a message to vehicles in range and return number of receipents.
        #print("[%s] Broadcasting message %s" % (sender.numberplate, message))

        # Accumulate number of bytes sent.
        self.bytes_sent[sender] = self.bytes_sent.get(sender, 0) + len(message)

        # Get receipents.
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

        # Ground truth.
        # Note that one numberplate can have multiple object ids.
        # This happens when a vehicle changes its EID.
        self._gt_eid_to_objectid: Dict[EID, int] = {}
        self._gt_objectid_to_eid: List[EID] = []
        self._gt_objectid_to_numberplate: List[NumberPlate] = []


    def perceive(self, ego: Vehicle) -> List[Tuple[int, Position, NumberPlate]]:
        # Return all perceived objects and their positions of the given egovehicle.
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

                # Record the ground truth if not already.
                # Object ID will be automatically assigned.
                if v.eid not in self._gt_eid_to_objectid:
                    self._gt_eid_to_objectid[v.eid] = len(self._gt_objectid_to_eid)
                    self._gt_objectid_to_eid.append(v.eid)
                    self._gt_objectid_to_numberplate.append(v.numberplate)

                ret.append((
                    self._gt_eid_to_objectid[v.eid],
                    v.position,
                    v.numberplate,
                ))

        return ret

class PerceptionSimulatorV3(object):
    ''' A more realistic perception simulator. '''
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles
        self.random = random_from(scenario.random)
        self.config = scenario.config['perception_simulator']
        self.vision_distance = self.config['vision_distance']
        self.fov = abs(self.config['fov']) * math.pi / 180
        self.target_max_rotation = abs(self.config['target_max_rotation']) * math.pi / 180

        self.position_manager = scenario.position_manager

        # Ground truth.
        # Note that one numberplate can have multiple object ids.
        # This happens when a vehicle changes its EID.
        self._gt_eid_to_objectid: Dict[EID, int] = {}
        self._gt_objectid_to_eid: List[EID] = []
        self._gt_objectid_to_numberplate: List[NumberPlate] = []

    def perceive(self, ego: Vehicle) -> List[Tuple[int, Position, NumberPlate]]:
        # Return all perceived objects and their positions of the given egovehicle.

        # Ego vehicle's position is the center of front bumper.
        camera = ego.position

        candidates = self.position_manager.get_nearby_vehicles(camera)
        candidates = [v for v in candidates if v != ego and v.position.distance_to(camera) < self.vision_distance]

        # Project all candidates on a number axis of the camera's viewing angle.
        candidate_positions = [v.position for v in candidates]
        candidate_lines = self.get_projected_lines_v2(
            camera,
            ego.config['length'],ego.config['width'], ego.config['numberplate_width'],
            candidate_positions,
        )

        # Filter out vehicles that are not in the camera's field of view.
        for r, v in zip(candidate_lines, candidates):
            r['vehicle'] = v
        candidate_lines = [r for r in candidate_lines if not (r['delta1'] > self.fov or r['delta2'] < -self.fov)]

        # Sort by distance.
        candidate_lines = sorted(candidate_lines, key=lambda r: r['dist'])

        # Filter out vehicles that are occluded by other vehicles.
        candidate_lines = self.get_visible_lines(camera, candidate_lines)

        # Filter out vehicles whose numberplates are too skewed.
        # This filtering should be done last because some "unidentifiable" vehicles may also occlude the others.
        candidate_lines = [r for r in candidate_lines if abs(r['gamma'] - r['beta']) < self.target_max_rotation]

        for r in candidate_lines:
            v = r['vehicle']
            # Record the ground truth if not already.
            # Object ID will be automatically assigned.
            if v.eid not in self._gt_eid_to_objectid:
                self._gt_eid_to_objectid[v.eid] = len(self._gt_objectid_to_eid)
                self._gt_objectid_to_eid.append(v.eid)
                self._gt_objectid_to_numberplate.append(v.numberplate)

        return [
            (self._gt_eid_to_objectid[r['vehicle'].eid], r['vehicle'].position, r['vehicle'].numberplate)
        for r in candidate_lines]

    def get_visible_lines(self, camera: Position, candidate_lines: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        # Calculate occlusion and return the visible vehicles in projection format.
        # The returned list is sorted by the distance to the camera.

        if not candidate_lines: return []

        all_points = sum([
            [i['delta1'], i['delta2'], i['rho1'], i['rho2']]
        for i in candidate_lines], [])
        tree = SegmentTree(all_points)

        ret = [candidate_lines[0]]

        for l, m in zip(candidate_lines[:-1], candidate_lines[1:]):
            tree.insert(l['delta1'], l['delta2'])
            if tree.query(m['rho1'], m['rho2']) == False:
                ret.append(m)

        return ret

    def get_projected_lines(self, camera: Position, length: float, width: float, numberplate_width: float, candidate_positions: List[Position]) -> List[dict]:
        # Get projected lines (Vehicle, diagnoal, numberplate) of all vehicles.
        # The lines are projected (straightened) on a number axis, representing the viewing angle [-pi, pi) from the camera.

        if not candidate_positions: return []

        x0, y0, beta0 = camera.x, camera.y, camera.heading * math.pi / 180

        # Get Position of center of front bumper of all candidates.
        x = np.array([p.x for p in candidate_positions], dtype=np.float32)
        y = np.array([p.y for p in candidate_positions], dtype=np.float32)
        beta = np.array([p.heading for p in candidate_positions], dtype=np.float32) * math.pi / 180

        # Convert the positions to the relative position of camera. The rotation is effectively **-beta0**.
        F = np.array([
            [ np.cos(beta0), np.sin(beta0)],
            [-np.sin(beta0), np.cos(beta0)],
        ]) @ np.array([x - x0, y - y0])

        # Convert the headings to the relative position of camera.
        beta = beta - beta0

        # ========================================
        #  Using camera reference frame from here
        # ========================================

        # Center.
        O = F - 0.5 * length * np.array([np.cos(beta), np.sin(beta)])

        # If b is not in the range of [-pi/2, pi/2], normalize it.
        # This effectively flips the vehicle along its heading.
        beta = np.mod(beta + math.pi / 2, math.pi) - math.pi / 2
        front = np.array([np.cos(beta), np.sin(beta)])
        perp = np.array([-np.sin(beta), np.cos(beta)])

        # Center of front and rear bumpers.
        F = O + 0.5 * length * front
        G = O - 0.5 * length * front

        # Four corners. A = left front, B = right front, C = right rear, D = left rear.
        A = F - 0.5 * width * perp
        B = F + 0.5 * width * perp
        C = G + 0.5 * width * perp
        D = G - 0.5 * width * perp

        # Numberplates. N and M should be on DC and D < M < N < C.
        M = G - 0.5 * numberplate_width * perp
        N = G + 0.5 * numberplate_width * perp

        # Angles of all corners and numberplates.
        delta1 = np.arctan2(A[1], A[0])
        delta2 = np.arctan2(B[1], B[0])
        delta3 = np.arctan2(C[1], C[0])
        delta4 = np.arctan2(D[1], D[0])
        delta_min = np.min([delta1, delta2, delta3, delta4], axis=0)
        delta_max = np.max([delta1, delta2, delta3, delta4], axis=0)

        rho1   = np.arctan2(M[1], M[0])
        rho2   = np.arctan2(N[1], N[0])
        rho_min = np.min([rho1, rho2], axis=0)
        rho_max = np.max([rho1, rho2], axis=0)

        # Distance and angle of G from camera.
        dist = np.linalg.norm(G, axis=0)
        gamma = np.arctan2(G[1], G[0])

        # Collect data.
        return [{
            'dist': dist[i],        # Distance of rear bumper.
            'beta': beta[i],        # Heading of vehicle.
            'gamma': gamma[i],      # Heading of rear bumper.
            'delta1': delta_min[i], # Leftmost angle of vehicle.
            'delta2': delta_max[i], # Rightmost angle of vehicle.
            'rho1': rho_min[i],     # Leftmost angle of numberplate.
            'rho2': rho_max[i],     # Rightmost angle of numberplate.
        } for i, v in enumerate(candidate_positions)]

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

