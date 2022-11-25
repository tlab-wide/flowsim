
from typing import *
import dataclasses
import random
import abc

from utils import *
from modules import *

@dataclasses.dataclass
class Vehicle(object):
    scenario: 'Scenario'
    numberplate: str
    config: object
    random: random.Random = random
    _is_malicious: bool = False

    position: Position = Position(np.nan, np.nan, np.nan)
    eid: EID = None
    
    # Define modules and data flow by adjacent table.
    # User must ensure the flow graph is connected and acyclic.
    # E.g.:
    #               LocalPerception ----> PoTProver -> CPSSender
    #                                 |
    #                                 v
    # CPSReceiver -> PoTVerifier -> Planner

    # Translate into:
    # [
    #     (LocalPerception, [PoTProver, Planner]),
    #     (PoTProver, [CPSSender]),
    #     (CPSSender, []),
    #     (CPSReceiver, [PoTVerifier]),
    #     (PoTVerifier, [Planner]),
    #     (Planner, []),
    # ]
    data_flow: List[Tuple[VehicleModule, List[VehicleModule]]] = None

    def __post_init__(self):
        # dataclass exposes this method which meant to be run after __init__.

        # Generate initial EID.
        self.change_eid()

        # Init modules.
        modules = list(i[0] for i in self.data_flow)
        assert len(modules) == len(set(modules)), "A module should appear only once from the left."

        self._module_instance_map = dict((i, i(self)) for i in modules)
        self._data_flow_map = dict(self.data_flow)

        # Find the source modules (which runs by itself with no input).
        # A source module is the one which never appears as the target of a flow.
        # Operates in the original list to maintain the right order of source modules.
        flow_targets = set(sum(self._data_flow_map.values(), []))
        self._source_modules = [i[0] for i in self.data_flow if i[0] not in flow_targets]

    def __eq__(lhs, rhs):
        return (
            lhs.__class__ == rhs.__class__ and 
            lhs.numberplate == rhs.numberplate
        )

    def __hash__(self):
        return hash(self.numberplate)

    def tick(self):
        # The upper level tick function.

        self.update_position()
        self.possibly_change_eid()
        self.do_work()

    def change_eid(self):
        self._eid = '%064x' % (self.random.randint(0, 2**256 - 1))

    def possibly_change_eid(self):
        if self.random.random() < self.config['eid_changing_possibility']:
            self.change_eid()

    def update_position(self):
        p = self.scenario.position_manager.get_vehicle_position(self)

    def do_work(self):
        def _dfs(self, module_class, input):
            module = self._module_instance_map(module_class)
            result = module.do_work()
            for i in self._data_flow_map[module]:
                _dfs(i, result)

        for i in self._source_modules:
            # DFS into the module flow tree.
            _dfs(i, None)

    def __hash__(self):
        return hash(self.numberplate)

# =========================================
#   Definition of different vehicle types
# =========================================

@dataclasses.dataclass
class UnconnectedVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [Planner]),
        (Planner, []),
    ]

@dataclasses.dataclass
class ConnectedVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [CPSSender, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner]),
        (Planner, []),
    ]

@dataclasses.dataclass
class PoTVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [PoTProver, Planner]),
        (PoTProver, [CPSSender]),
        (CPSSender, []),
        (CPSReceiver, [PoTVerifier]),
        (PoTVerifier, [Planner]),
        (Planner, []),
    ]

@dataclasses.dataclass
class MaliciousVehicle(Vehicle):
    _is_malicious = True

@dataclasses.dataclass
class SpamAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [CPSSender, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner]),
        (CPSSpammer, []),
        (Planner, []),
    ]

@dataclasses.dataclass
class ReplayAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [CPSSender, CPSReplayer, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner, CPSReplayer]),
        (CPSReplayer, []),
        (Planner, []),
    ]

@dataclasses.dataclass
class SilenceAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [Planner]),
        (CPSReceiver, [PoTVerifier]),
        (PoTVerifier, [Planner]),
        (Planner, []),
    ]

@dataclasses.dataclass
class SybilAttacker(MaliciousVehicle):
    pass

