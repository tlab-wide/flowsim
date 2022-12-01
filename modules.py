
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
            perceived_objects = [(oid, pos) for oid, pos, num in new_objects],
            _objid_to_numberplate = dict((oid, num) for oid, pos, num in new_objects),
        )

class Planner(VehicleModule):
    # Planning module

    def __init__(self, vehicle: 'Vehicle'):

        super().__init__(vehicle)

        self.obj_seen: List[set] = []

        self.then = -1

    def do_work(self, input: CPM) -> None:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        now = self.vehicle.scenario.now

        if self.then != now:
            self.then = now
            self.obj_seen.insert(0, set())
            if len(self.obj_seen) > 10:
                self.obj_seen.pop()

        for objid, pos in input.perceived_objects:
            self.obj_seen[0].add(objid)

        return None

    def recent_seen_objids(self) -> Set[int]:
        return functools.reduce(lambda x, y: x.union(y), self.obj_seen, set())

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
        self.known_numberplates.union(input._objid_to_numberplate.values())

        # Match!
        self.update_matches()

        # Generate proofs for all matched vehicles.

        # TODO: queue excessive proofs.
        assert input.proofs == []

        for objid, pos in input.perceived_objects:
            numberplate = input._objid_to_numberplate[objid]
            eid = self._numberplate_to_eid.get(numberplate, None)

            if not eid: # No match.
                continue

            input.proofs.append((
                objid,
                pot_proof(numberplate, eid, self.vehicle.eid),
            ))

        input._objid_to_numberplate = {}

        #print("%s: #match = %d, #proof = %s" % (
        #    self.vehicle.numberplate,
        #    len(self._eid_to_numberplate),
        #    len(input.proofs),
        #))

        return input

class PoTVerifier(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):

        super().__init__(vehicle)

        self.pubkey_to_provers: Dict[Pubkey, set] = {}

        self._unconfirmed_objects: Dict['Vehicle', Position] = {}

    def do_work(self, input: CPM) -> CPM:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        if input.proofs:
            import pdb; pdb.set_trace()

        # Generate pubkey from proofs and store them.
        objid_to_pubkey: Dict[int, Pubkey] = {}
        for objid, p in input.proofs:
            pubkey = utils.pot_pubkey(p, input.sender)

            if pubkey == None:
                # It is not a valid proof.
                continue

            objid_to_pubkey[objid] = pubkey

            if pubkey not in self.pubkey_to_provers:
                self.pubkey_to_provers[pubkey] = set()
            
            self.pubkey_to_provers[pubkey].add(input.sender)

        # Stage objects.
        for oid, pos in input.perceived_objects:
            self._unconfirmed_objects[oid] = pos

        # Filter confirmed and unconfirmed objects.
        confirmed = []
        unconfirmed = []

        for oid, pos in self._unconfirmed_objects.items():
            # The Object didn't come with a proof, ignore.
            if oid not in objid_to_pubkey:
                unconfirmed.append((oid, pos))
                continue

            provers = self.pubkey_to_provers.get(objid_to_pubkey[oid], set())

            if len(provers) < 2:
                unconfirmed.append((oid, pos))
                continue

            confirmed.append((oid, pos))

        self._unconfirmed_objects = dict(unconfirmed)

        #print("%s: #confirmed = %d, #unconfirmed = %d" % (
        #    self.vehicle.numberplate,
        #    len(confirmed),
        #    len(unconfirmed),
        #))

        return CPM(input.sender, confirmed)
