
from typing import *
import atexit
import math
import time

from vehicle import *
from modules import *
from utils import *
import traci

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
        self.vehicles: Dict[NumberPlate, Vehicle] = {}

        # Create simulator modules.
        self.position_manager = PositionManagerV2(self)
        self.network = NetworkSimulator(self)
        self.perception = PerceptionSimulatorV3(self)
        self.match = MatchSimulator(self)

        self.i = 0

        # Create metric collectors.
        os.makedirs(self.output_dir, exist_ok=True)
        format_dir = lambda x: os.path.join(self.output_dir, x)
        collect_per_vehicle = lambda func: lambda: {vid: func(v) for vid, v in self.vehicles.items()}

        self.metric_collectors = {
            'x': MetricCollector(format_dir('x.json'), collect_per_vehicle(lambda v: "%.3f" % v.position.x)),
            'y': MetricCollector(format_dir('y.json'), collect_per_vehicle(lambda v: "%.3f" % v.position.y)),

            #'recent_saw_by': MetricCollector(format_dir('recent_saw_by.json'), self.collect_recent_saw_by),
            'bytes_sent': MetricCollector(format_dir('bytes_sent.json'), self.collect_vehicle_sent_bytes),

            'sent_proofs':     MetricCollector(format_dir('sent_proofs.json'    ), collect_per_vehicle(lambda v: v.n_sent_proofs)),
            'enqueued_proofs': MetricCollector(format_dir('enqueued_proofs.json'), collect_per_vehicle(lambda v: v.n_enqueued_proofs)),
            'dropped_proofs':  MetricCollector(format_dir('dropped_proofs.json' ), collect_per_vehicle(lambda v: v.n_dropped_proofs)),

            'all_objects':      MetricCollector(format_dir('all_objects.json'     ), collect_per_vehicle(lambda v: len(v.all_objects     ))),
            'local_objects':    MetricCollector(format_dir('local_objects.json'   ), collect_per_vehicle(lambda v: len(v.local_objects   ))),
            'received_objects': MetricCollector(format_dir('received_objects.json'), collect_per_vehicle(lambda v: len(v.received_objects))),

            'time_to_verify_histogram': MetricCollector(format_dir('ttv_histogram.json'), collect_per_vehicle(lambda v: v.time_to_verify_buckets)),
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

        self.traci.start([sumo_bin, "--no-warnings", "true", "-c", self.config['sumo_config_path']])

        self.now = self.traci.simulation.getTime()
        self.end_time = self.traci.simulation.getEndTime()

        print("Initialized sumo with end time: %s" % self.end_time)

    def cleanup(self):
        [mc.save() for mc in self.metric_collectors.values()]

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

    def tick(self, preheat = False):
        t0 = time.time()
        # Run one traci step first.
        self.traci.simulationStep()

        self.now = traci.simulation.getTime()
        if self.now >= self.end_time: raise StopIteration

        # Update vehicle list from traci.
        dead_vehicles = self.vehicles.keys() & set(self.traci.simulation.getArrivedIDList())
        for vid in dead_vehicles:
            # The dead vehicles will be automatically unsubscribed.
            #self.traci.vehicle.unsubscribe(vid)
            self.vehicles[vid].cleanup()
            del self.vehicles[vid]

        born_vehicles = set(self.traci.simulation.getDepartedIDList()) - self.vehicles.keys()
        for vid in born_vehicles:
            self.vehicles[vid] = self.generate_vehicle(vid)
            self.traci.vehicle.subscribe(vid, [traci.constants.VAR_POSITION, traci.constants.VAR_ANGLE])

        if preheat: return 

        # Give vehicles chance to thange their EIDs and let match simulator know.
        self.match.update_eids()

        t1 = time.time()
        # Let position manager to update vehicles' position.
        self.position_manager.update_all_position()
        [v.update_position() for v in self.vehicles.values()]

        t2 = time.time()
        self.perception.update_all_perception()

        t3 = time.time()
        [v.do_work() for v in self.vehicles.values()]

        t4 = time.time()
        [i.collect() for i in self.metric_collectors.values()]

        t5 = time.time()
        # Print out some statistics.
        format_time = lambda dt: "%3.0f ms(%3.0f us)" % (dt * 1000, dt * 1000 * 1000 / len(self.vehicles))
        log_ = "step: %s; pos: %s; perception: %s; work: %s; coll: %s; total: %s; tick: %.0f; #vehicles: %d" % (
            format_time(t1 - t0),
            format_time(t2 - t1),
            format_time(t3 - t2),
            format_time(t4 - t3),
            format_time(t5 - t4),
            format_time(t5 - t0),
            self.now,
            len(self.vehicles),
        )

        self.i += 1
        if self.i % 10 == 0:
            print(log_)

    # Metric collectors.
    def collect_vehicle_sent_bytes(self) -> VehicleMetric:
        # Collect how many bytes a vehicle sent in this tick.
        ret = {}

        for v, b in self.network.bytes_sent.items():
            ret.setdefault(v.numberplate, 0)
            ret[v.numberplate] += b

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
            for objid in module.get_recent_seen_objids():
                try:
                    numberplate = self.perception._gt_objectid_to_numberplate[objid]
                    ret[numberplate] += 1
                except (IndexError, KeyError):
                    # IndexError is for gt_objectid_to_numberplate; KeyError is for ret.
                    ret[UNKNOWN_PLATE] += 1

        return ret

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
        grid_x = math.ceil(self.boundary[1][0] / self.range_limit) + 4
        grid_y = math.ceil(self.boundary[1][1] / self.range_limit) + 4

        self._grid = [[set() for _ in range(grid_y)] for _ in range(grid_x)]

        # Nearby vehicles of a position.
        self._nearby_vehicles = {}

        print("PositionManagerV2 initialized with boundary %s, grid size %d x %d" % (self.boundary, grid_x, grid_y))

    def get_vehicle_position(self, vehicle: Vehicle) -> Position:
        return self._position[vehicle]

    def get_nearby_vehicles(self, position: Position) -> List[Vehicle]:
        #return self._nearby_vehicles.get(position, [])
        if position.x == -1073741824:
            # This is the teleportation target.
            return []
        return self._nearby_vehicles[position]

    def _update_nearby(self):
        # Update nearby vehicles for position of all vehicles.
        ret = []

        for ego, position in self._position.items():
            # Offset by two since the grid is padded.
            grid_x = math.floor(position.x / self.range_limit) + 2
            grid_y = math.floor(position.y / self.range_limit) + 2

            # Do nothing if the position is out of range.
            if not 0 < grid_x <= self.boundary[1][0] or not 0 < grid_y <= self.boundary[1][1]:
                continue

            # Get all vehicles in the 9 adjacent grids as candidates.
            candidates = set.union(*[self._grid[grid_x + i][grid_y + j] for i in [-1, 0, 1] for j in [-1, 0, 1]])

            # Filter out ego and vehicles > range_limit away.
            results = [v for v in candidates if self._position[v].distance_to(position) < self.range_limit and v != ego]

            ret.append((position, results))

        self._nearby_vehicles = dict(ret)

    def update_all_position(self):
        # Set vehicles' positions to the data at the given tick.
        self._position = {}

        result = self.traci.vehicle.getAllSubscriptionResults()

        for vid, v in self.vehicles.items():
            x, y = result[vid][traci.constants.VAR_POSITION]
            yaw  = result[vid][traci.constants.VAR_ANGLE]

            # Warning: getAngle returns yaw (0 for North, 90 for East, etc.)
            # Need to convert it to theta (0 for East, 90 for North, etc.)
            theta = (360 + 90 - yaw) % 360
            self._position[v] = Position(x, y, theta)

        self._update_grid()
        self._update_nearby()

    def _update_grid(self):
        # Recreate grid.
        self._grid = [[set() for _ in range(len(self._grid[0]))] for _ in range(len(self._grid))]

        for v, pos in self._position.items():
            # Offset by two since the grid is padded.
            grid_x = math.floor(pos.x / self.range_limit) + 2
            grid_y = math.floor(pos.y / self.range_limit) + 2

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

        self.perceived_objects = {}

        # Ground truth.
        # Note that one numberplate can have multiple object ids.
        # This happens when a vehicle changes its EID.
        self._gt_eid_to_objectid: Dict[EID, int] = {}
        self._gt_objectid_to_eid: List[EID] = []
        self._gt_objectid_to_numberplate: List[NumberPlate] = []

    def update_all_perception(self):
        ret = []
        for v in self.vehicles.values():
            ret.append((v, self._perceive(v)))

        self.perceived_objects = dict(ret)

    def perceive(self, ego: Vehicle) -> List[Tuple[int, Position, NumberPlate]]:
        # Return all perceived objects and their positions of the given egovehicle.
        return self.perceived_objects[ego]

    def _perceive(self, ego: Vehicle) -> List[Tuple[int, Position, NumberPlate]]:
        # Return all perceived objects and their positions of the given egovehicle.

        # Ego vehicle's position is the center of front bumper.
        camera = ego.position

        candidates = self.position_manager.get_nearby_vehicles(camera)
        candidates = [v for v in candidates if v != ego and v.position.distance_to(camera) < self.vision_distance]

        # Project all candidates on a number axis of the camera's viewing angle.
        candidate_positions = [v.position for v in candidates]
        candidate_lines = self.get_projected_lines(
            camera,
            ego.config['length'],ego.config['width'], ego.config['numberplate_width'],
            candidate_positions,
        )
        for r, v in zip(candidate_lines, candidates): r['vehicle'] = v

        # Filter out vehicles that are not in the camera's field of view.
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

        return [(
            self._gt_eid_to_objectid[r['vehicle'].eid],
            r['vehicle'].position,
            r['vehicle'].numberplate,
        ) for r in candidate_lines]

    def get_visible_lines(self, camera: Position, candidate_lines: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        # Calculate occlusion and return the visible vehicles in projection format.
        # The returned list is sorted by the distance to the camera.
        if not candidate_lines: return []

        all_points = sum([[i['delta1'], i['delta2'], i['rho1'], i['rho2']] for i in candidate_lines], [])
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
        cosbeta0, sinbeta0 = math.cos(beta0), math.sin(beta0)

        ret = []
        for i in candidate_positions:
            # Get Position of center of front bumper of all candidates.
            # Convert the positions and headings to the relative position of camera. The rotation is effectively **-beta0**.
            x, y, beta = i.x, i.y, i.heading * math.pi / 180
            F = (
                 cosbeta0 * (x - x0) + sinbeta0 * (y - y0),
                -sinbeta0 * (x - x0) + cosbeta0 * (y - y0),
            )
            beta -= beta0

            # ========================================
            #  Using camera reference frame from here
            # ========================================

            # Center.
            O = (F[0] - 0.5 * length * math.cos(beta), F[1] - 0.5 * length * math.sin(beta))

            # If b is not in the range of [-pi/2, pi/2], normalize it.
            # This effectively flips the vehicle along its heading.
            beta = (beta + math.pi / 2) % math.pi - math.pi / 2
            front = (math.cos(beta), math.sin(beta))
            perp = (-math.sin(beta), math.cos(beta))

            # Center of front and rear bumpers.
            F = (O[0] + 0.5 * length * front[0], O[1] + 0.5 * length * front[1])
            G = (O[0] - 0.5 * length * front[0], O[1] - 0.5 * length * front[1])

            # Four corners. A = left front, B = right front, C = right rear, D = left rear.
            A = (F[0] - 0.5 * width * perp[0], F[1] - 0.5 * width * perp[1])
            B = (F[0] + 0.5 * width * perp[0], F[1] + 0.5 * width * perp[1])
            C = (G[0] + 0.5 * width * perp[0], G[1] + 0.5 * width * perp[1])
            D = (G[0] - 0.5 * width * perp[0], G[1] - 0.5 * width * perp[1])

            # Numberplates. N and M should be on DC and D < M < N < C.
            M = (G[0] - 0.5 * numberplate_width * perp[0], G[1] - 0.5 * numberplate_width * perp[1])
            N = (G[0] + 0.5 * numberplate_width * perp[0], G[1] + 0.5 * numberplate_width * perp[1])

            # Angles of all corners and numberplates.
            delta1 = math.atan2(A[1], A[0])
            delta2 = math.atan2(B[1], B[0])
            delta3 = math.atan2(C[1], C[0])
            delta4 = math.atan2(D[1], D[0])
            delta_min = min(delta1, delta2, delta3, delta4)
            delta_max = max(delta1, delta2, delta3, delta4)

            rho1 = math.atan2(M[1], M[0])
            rho2 = math.atan2(N[1], N[0])
            rho_min = min(rho1, rho2)
            rho_max = max(rho1, rho2)

            # Distance and angle of G from camera.
            dist = math.hypot(G[0], G[1])
            gamma = math.atan2(G[1], G[0])

            # Collect data.
            ret.append({
                'dist': dist,        # Distance to the center rear bumper.
                'beta': beta,        # Heading of vehicle.
                'gamma': gamma,      # Angle of the center of rear bumper.
                'delta1': delta_min, # Leftmost angle of vehicle.
                'delta2': delta_max, # Rightmost angle of vehicle.
                'rho1': rho_min,     # Leftmost angle of numberplate.
                'rho2': rho_max,     # Rightmost angle of numberplate.
            })

        return ret

class MatchSimulator(object):
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.vehicles = scenario.vehicles

        self.eid_to_numberplate: Dict[EID, NumberPlate] = {}

    def update_eids(self):
        # Update EIDs of all vehicles.
        # Only keep the latest EID of each vehicle.
        self.eid_to_numberplate = {v.eid: vid for vid, v in self.vehicles.items()}

    def match_eid(self, known_numberplates: Set[NumberPlate], eid: EID) -> NumberPlate:
        # Match a EID to a numberplate in candidates.
        # Return the number plate if found, otherwise return None.
        ret = self.eid_to_numberplate.get(eid, None)
        return ret if ret in known_numberplates else None
