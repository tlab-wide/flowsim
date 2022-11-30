
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
    config: dict
    random: random.Random = random

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
    data_flow = None

    # A flag to (internally) indicate whether the vehicle is a malicious one.
    _is_malicious = False

    def __post_init__(self):
        # dataclass exposes this method which meant to be run after __init__.

        # Generate initial EID.
        self.change_eid()

        # Init modules.
        data_flow = self.__class__.data_flow

        modules = list(i[0] for i in data_flow)
        assert len(modules) == len(set(modules)), "A module should appear only once from the left."

        self._module_instance_map = dict((i, i(self)) for i in modules)
        self._data_flow_map = dict(data_flow)

        # Find the source modules (which runs by itself with no input).
        # A source module is the one which never appears as the target of a flow.
        # Operates in the original list to maintain the right order of source modules.
        flow_targets = set(sum(self._data_flow_map.values(), []))
        self._source_modules = [i[0] for i in data_flow if i[0] not in flow_targets]

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
        self.eid = '%064x' % (self.random.randint(0, 2**256 - 1))

    def possibly_change_eid(self):
        if self.random.random() < self.config['eid_changing_possibility']:
            self.change_eid()

    def update_position(self):
        self.position = self.scenario.position_manager.get_vehicle_position(self)

    def get_module(self, module_class: Type['Module']) -> 'Module':
        return self._module_instance_map.get(module_class, None)

    def do_work(self):
        def _dfs(module_class, input):
            module = self.get_module(module_class)
            output = module.do_work(input)

            # Break dfs if the current module produces no output.
            if output == None:
                return

            # Handle list of output as well.
            if type(output) != list:
                output = [output]

            for o in output:
                assert isinstance(o, CPM), \
                    f"Module {module_class.__name__} should return a CPM instead of {type(result)}."

                for target in self._data_flow_map[module_class]:
                    _dfs(target, o)


        for i in self._source_modules:
            # DFS into the module flow tree.
            _dfs(i, None)

    def __hash__(self):
        return hash(self.numberplate)

# =========================================
#   Definition of different vehicle types
# =========================================

class UnconnectedVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [Planner]),
        (Planner, []),
    ]

class ConnectedVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [CPSSender, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner]),
        (Planner, []),
    ]

class PoTVehicle(Vehicle):
    data_flow = [
        (LocalPerception, [PoTProver, Planner]),
        (PoTProver, [CPSSender]),
        (CPSSender, []),
        (CPSReceiver, [PoTProver, PoTVerifier]),
        (PoTVerifier, [Planner]),
        (Planner, []),
    ]

class MaliciousVehicle(Vehicle):
    _is_malicious = True

class SpamAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [CPSSender, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner]),
        (CPSSpammer, []),
        (Planner, []),
    ]

class ReplayAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [CPSSender, CPSReplayer, Planner]),
        (CPSSender, []),
        (CPSReceiver, [Planner, CPSReplayer]),
        (CPSReplayer, []),
        (Planner, []),
    ]

class SilenceAttacker(MaliciousVehicle):
    data_flow = [
        (LocalPerception, [Planner]),
        (CPSReceiver, [PoTVerifier]),
        (PoTVerifier, [Planner]),
        (Planner, []),
    ]

class SybilAttacker(MaliciousVehicle):
    pass

