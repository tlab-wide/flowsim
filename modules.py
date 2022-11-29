
import abc
import copy
import functools
import dataclasses

from utils import *
import utils

class VehicleModule(metaclass=abc.ABCMeta):
    # A module in a Vehicle 

    def __init__(self, vehicle: 'Vehicle'):
        self.vehicle = vehicle

    @abc.abstractmethod
    def do_work(self, input: CPM) -> CPM:
        return input


class LocalPerception(VehicleModule):
    # Local perception module

    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        new_objects = self.vehicle.scenario.perception.perceive(self.vehicle)

        if len(new_objects) > 128:
            # TODO: we need a queue.
            raise NotImplementedError

        return CPM(
            sender=self.vehicle.eid,
            perceived_objects = [(None, v, v.position) for v in new_objects],
        )

class Planner(VehicleModule):
    # Planning module

    def __init__(self, vehicle: 'Vehicle'):

        super().__init__(vehicle)

        self.vehicle_seen: List[set] = []

        self.then = -1

    def do_work(self, input: CPM) -> None:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        now = self.vehicle.scenario.now

        if self.then != now:
            self.then = now
            self.vehicle_seen.insert(0, set())
            if len(self.vehicle_seen) > 10:
                self.vehicle_seen.pop()

        for _, v, pos in input.perceived_objects:
            self.vehicle_seen[0].add(v)

        return None

    def recent_seen_vehicles(self) -> Set['Vehicle']:
        return functools.reduce(lambda x, y: x.union(y), self.vehicle_seen, set())

class CPSReceiver(VehicleModule):
    def do_work(self, input: None) -> List[CPM]:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        return self.vehicle.scenario.network.receive(self.vehicle)
        

class CPSSender(VehicleModule):
    def do_work(self, input: CPM) -> None:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        input.verify()

        self.vehicle.scenario.network.broadcast(self.vehicle, input)
        return None

class CPSSpammer(VehicleModule):
    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        raise NotImplementedError

class CPSReplayer(VehicleModule):
    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        raise NotImplementedError


class PoTProver(VehicleModule):

    def __init__(self, vehicle: 'Vehicle'):
        super().__init__(vehicle)

        # Known numberplates.
        self.known_numberplates: Set[NumberPlate] = set()

        # EIDs left unmatched.
        self.unmatched_eids: Set[EID] = set()

        # Two-way Numberplate to EID mapping.
        # Only latest EID of a Numberplate is recorded.
        self._eid_to_numberplate: Dict[EID, NumberPlate] = {}
        self._numberplate_to_eid: Dict[NumberPlate, EID] = {}

    def update_matches(self):
        # Update match repository.

        for e in list(self.unmatched_eids):
            n = self.vehicle.scenario.match.match_eid(self.known_numberplates, e)

            if n == None: # No match.
                continue

            self.unmatched_eids.remove(e)

            self._eid_to_numberplate[e] = n
            self._numberplate_to_eid[n] = e

    def do_work(self, input: CPM) -> CPM:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        # Notice modules runs after change_eid(),
        # so the ego's EID is consistent among modules,
        # and can be used for detemine source of CPM.
        if input.sender != self.vehicle.eid:
            # Received a CPM from another vehicle.
            # We only need to collect its EID.

            self.unmatched_eids.add(input.sender)

            self.update_matches()

            return None

        # Received a CPM from LocalPerception.

        # Record numberplates from perceived objects.
        for _, v, _ in input.perceived_objects:
            self.known_numberplates.add(v.numberplate)
                
        # Match!
        self.update_matches()

        # Generate proofs for all matched vehicles.

        # TODO: queue excessive proofs.
        assert input.proofs == []

        for objid, v, _ in input.perceived_objects:
            if v.numberplate in self._numberplate_to_eid:
                input.proofs.append((
                    objid,
                    pot_proof(v.eid, v.numberplate, self.vehicle.eid),
                ))

        print("%s: #match = %d, proof = %s" % (
            self.vehicle.numberplate,
            len(self._eid_to_numberplate),
            input.proofs,
        ))

        return input

class PoTVerifier(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):

        super().__init__(vehicle)

        self.confirmed_eids = set()

        self.confirmed_proofs: Dict[Pubkey, EID] = {}   # value: Target EID
        self.unconfirmed_proofs: Dict[Pubkey, EID] = {} # value: Sender EID

        self._unconfirmed_objects: Dict['Vehicle', Position] = {}

    def do_work(self, input: CPM) -> CPM:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        self.update_proof_db(input)

        self.stage_objects(input)

        return self.flush_objects(input)

    def _get_obj_by_objid(self, input: CPM, objid: int) -> 'Vehicle':
        for o, v, _ in input.perceived_objects:
            if o == objid:
                return v
        raise ValueError("Object not found in CPM")

    def update_proof_db(self, input: CPM):
        for objid, p in input.proofs:
            pubkey = utils.pot_pubkey(p)

            # If the pubkey is already confirmed, do nothing.
            if pubkey in self.confirmed_proofs:
                continue

            # If the pubkey is seen for the first time, add it to unconfirmed.
            if pubkey not in self.unconfirmed_proofs:
                self.unconfirmed_proofs[pubkey] = input.sender

            if input.sender != self.unconfirmed_proofs[pubkey]:
                # We found a confirmed proof!
                target_id = self._get_obj_by_objid(input, objid).eid

                self.confirmed_eids.add(target_id)

                self.confirmed_proofs[pubkey] = target_id
                del self.unconfirmed_proofs[pubkey]

    def stage_objects(self, input: CPM):
        # XXX: Actually, we don't need to stage unconfirmed objects in this implementation,
        # since we assume that the proofs always comes along with the corresponding fresh objects.
        for o, v, pos in input.perceived_objects:
            self._unconfirmed_objects[v] = pos

    def flush_objects(self, input: CPM) -> CPM:

        confirmed_objects_list = [
            (None, v, pos) for v, pos in self._unconfirmed_objects.items() if
            v.eid in self.confirmed_eids
        ]

        # Update still unconfirmed objects.
        self._unconfirmed_objects = dict(
            (v, pos) for v, pos in self._unconfirmed_objects.items() if
            v.eid not in self.confirmed_eids
        )

        return CPM(input.sender, confirmed_objects_list)
