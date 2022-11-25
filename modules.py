
import dataclasses
import abc
import copy

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

    def do_work(self, input: CPM) -> CPM:

        new_objects = self.vehicle.scenario.perception.perceive(self)

        if len(new_objects) > 128:
            # TODO: we need a queue.
            raise NotImplementedError

        ret = CPM(perceived_objects = [
            (None, v, v.position) for v in new_objects
        ])

class Planner(VehicleModule):
    # Planning module
    def do_work(self, input: CPM) -> CPM:
        return None

class CPSReceiver(VehicleModule):
    def do_work(self, input: CPM) -> CPM:
        return self.vehicle.scenario.network.receive(self.vehicle)
        

class CPSSender(VehicleModule):
    def do_work(self, input: CPM) -> CPM:

        cpm.verify()
        self.vehicle.scenario.network.broadcast(self.vehicle, CPM)
        return None

class CPSSpammer(VehicleModule):
    def do_work(self, input: CPM) -> CPM:
        raise NotImplementedError

class CPSReplayer(VehicleModule):
    def do_work(self, input: CPM) -> CPM:
        raise NotImplementedError

class PoTProver(VehicleModule):
    def do_work(self, input: CPM) -> CPM:
        # TODO: queue excessive proofs.
        assert input.proofs == []

        self_id = self.vehicle.eid

        proofs = [
            (objid, pot_proof(v.eid, v.numberplate, self.id))
            for objid, v, _ in input.perceived_objects
        ]

        return CPM(self_id, input.perceived_objects, proofs)

class PoTVerifier(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):
        super().__init__(vehicle)

        self.confirmed_eids = set()

        self.confirmed_proofs: Dict[Pubkey, EID] = {}   # value: Target EID
        self.unconfirmed_proofs: Dict[Pubkey, EID] = {} # value: Sender EID

        self._unconfirmed_objects: List[Tuple['Vehicle', Position]] = []

    def do_work(self, input: CPM) -> CPM:
        self.update_proof_db(input)

        self.stage_objects(input)

        return self.flush_objects(input)

    def _get_obj_by_objid(input: CPM, objid: int) -> 'Vehicle':
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
                self._proof_db[pubkey] = input.sender

            if input.sender != self._proof_db[pubkey]:
                # We found a confirmed proof!
                target_id = self._get_obj_by_objid(input).eid

                self.confirmed_eids.add(target_id)

                self.confirmed_proofs[pubkey] = target_id
                del self.unconfirmed_proofs[pubkey]

    def stage_objects(self, input: CPM):
        # XXX: Actually, we don't need to stage unconfirmed objects in this implementation,
        # since we assume that the proofs always comes along with the corresponding fresh objects.
        for o, v, pos in input.perceived_objects:
            self._unconfirmed_objects[v] = pos

    def flush_objects(self, input: CPM) -> CPM:

        confirmed_objects = [
            (None, v, pos) for v, pos in self._unconfirmed_objects if
            v.eid in self.confirmed_eids
        ]

        # Update still unconfirmed objects.
        self._unconfirmed_objects = [
            (v, pos) for v, pos in self._unconfirmed_objects if
            v.eid not in self.confirmed_eids
        ]

        return CPM(input.sender, confirmed_objects)
