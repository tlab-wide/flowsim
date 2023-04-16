
import sys
import abc
import dataclasses

from utils import *

class VehicleModule(metaclass=abc.ABCMeta):
    # A module in a Vehicle 
    def __init__(self, vehicle: 'Vehicle'):
        self.vehicle = vehicle

    @abc.abstractmethod
    def do_work(self, input: CPM) -> CPM:
        return input

    def flush(self) -> None:
        pass

# =============================================================================
#                              Basic modules
# =============================================================================

class LocalPerception(VehicleModule):
    # Local perception module
    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        new_objects = self.vehicle.scenario.perception.perceive(self.vehicle)

        self.vehicle.local_objects.update([i[0] for i in new_objects])
        
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
        self.recent_seen_objects = RingBuffer(10, lambda: set())

    def do_work(self, input: CPM) -> None:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        objids = [i[0] for i in input.perceived_objects]
        self.recent_seen_objects.current.update(objids)
        self.vehicle.all_objects.update(objids)

        return None

    def get_recent_seen_objids(self) -> Set[int]:
        return set.union(self.recent_seen_objects.data)

    def flush(self) -> None:
        # Renew the recent_seen_objects.
        self.recent_seen_objects.advance()

class CPSReceiver(VehicleModule):
    def do_work(self, input: None) -> List[CPM]:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        cpms = self.vehicle.scenario.network.receive(self.vehicle)

        for cpm in cpms:
            self.vehicle.received_objects.update([i[0] for i in cpm.perceived_objects])

        return cpms

class CPSSender(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):
        super().__init__(vehicle)

        self.cpms_to_send: List[CPM] = []

    def do_work(self, input: CPM) -> None:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        # Canonicalize the CPM.
        input.sender = self.vehicle.eid
        input._objid_to_numberplate = {}

        self.cpms_to_send.append(input)

        return None

    def flush(self) -> None:
        # Join all CPMs generated in this tick and flush it to the network.
        cpm = sum(self.cpms_to_send, CPM(sender=self.vehicle.eid))
        cpm.verify()
        self.cpms_to_send = []

        if len(cpm.perceived_objects) == 0 and len(cpm.proofs) == 0: return

        self.vehicle.scenario.network.broadcast(self.vehicle, cpm)

# =============================================================================
#                                 PoT modules
# =============================================================================

class PoTProver(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):
        super().__init__(vehicle)

        # Initialize config.
        self.config = self.vehicle.config.get('PoTProver', {})
        self.max_queued_proofs = self.config.get('max_queued_proofs', 0)
        self.send_proof_every = self.config.get('send_proof_every', 1)

        # Known numberplates.
        self.known_numberplates: Set[NumberPlate] = set()

        # EIDs left unmatched.
        self.recent_unmatched_eids = RingBuffer(10, lambda: set())

        # Two-way Numberplate to EID mapping.
        # Only latest EID of a Numberplate is recorded.
        self._eid_to_numberplate: Dict[EID, NumberPlate] = {}
        self._numberplate_to_eid: Dict[NumberPlate, EID] = {}

        # Store proofs sent in last n ticks.
        self.recent_sent_proofs = RingBuffer(self.send_proof_every, lambda: set())

        self.queued_proofs: List[Tuple[int, EID]] = []

    def update_matches(self):
        # Update match repository against recent unmatched EIDs.
        for s in self.recent_unmatched_eids.data:
            to_remove = []
            for e in s:
                n = self.vehicle.scenario.match.match_eid(self.known_numberplates, e)

                if n == None: # No match.
                    continue

                to_remove.append(e)

                self._eid_to_numberplate[e] = n
                self._numberplate_to_eid[n] = e

            for e in to_remove:
                s.remove(e)

    def generate_proofs(self, input: CPM) -> CPM:
        # Generate proofs for all matched vehicles in the given CPM.
        for objid, pos in input.perceived_objects:
            numberplate = input._objid_to_numberplate[objid]
            eid = self._numberplate_to_eid.get(numberplate, None)

            if not eid: # No match.
                continue

            if any(numberplate in i for i in self.recent_sent_proofs.data):
                # Do not send duplicate proofs.
                continue

            proof_entry = (objid, pot_proof(numberplate, eid, self.vehicle.eid))

            if len(input.proofs) >= 8:
                # Queue excessive proofs.
                if len(self.queued_proofs) >= self.max_queued_proofs:
                    # Drop the oldest proof if queue is full.
                    droped_objid = self.queued_proofs.pop(0)[0]
                    self.vehicle.n_dropped_proofs += 1
                    print("[%s] Warning: drop oldest queued proof" % self.vehicle.numberplate, file=sys.stderr)

                self.queued_proofs.append(proof_entry)
                self.vehicle.n_enqueued_proofs += 1
                continue

            input.proofs.append(proof_entry)

            # Record proofs sent in this tick.
            self.recent_sent_proofs.current.add(numberplate)

        # If we have spaces for more proofs, fill them with queued proofs.
        while len(input.proofs) < 8 and len(self.queued_proofs) > 0:
            proof_entry = self.queued_proofs.pop(0)
            input.proofs.append(proof_entry)
            self.recent_sent_proofs.current.add(proof_entry[0])

        self.vehicle.n_sent_proofs = len(input.proofs)

        input._objid_to_numberplate = {}

        return input

    def hear(self, e: EID) -> None:
        # Hear from a vehicle.

        # If the sender is matched, do nothing.
        if e in self._eid_to_numberplate: return

        # Try match it against known numberplates.
        n = self.vehicle.scenario.match.match_eid(self.known_numberplates, e)

        if n:
            self._eid_to_numberplate[e] = n
            self._numberplate_to_eid[n] = e
        else:
            # Only keep unmatched EIDs.
            self.recent_unmatched_eids.current.add(e)

    def do_work(self, input: CPM) -> CPM:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        # Notice that modules runs after change_eid(),
        # so the ego's EID is consistent among modules,
        # and can be used for determine source of CPM.
        if input.sender != self.vehicle.eid:
            # Received a CPM from another vehicle.
            # We only need to collect its EID.
            return self.hear(input.sender)

        # Received a CPM from LocalPerception.

        # Record numberplates from perceived objects.
        self.known_numberplates |= set(input._objid_to_numberplate.values())

        # Match!
        self.update_matches()

        return self.generate_proofs(input)

    def flush(self) -> None:
        # Advance the recent_sent_proofs and recent_unmatched_eids.
        self.recent_sent_proofs.advance()
        self.recent_unmatched_eids.advance()

class PoTVerifier(VehicleModule):
    def __init__(self, vehicle: 'Vehicle'):
        super().__init__(vehicle)

        self.pubkey_to_provers: Dict[Pubkey, set] = {}
        self._unconfirmed_objects: Dict[int, Position] = {}

    def do_work(self, input: CPM) -> CPM:
        assert isinstance(input, CPM), f"Module {self.__class__.__name__} requires CPM input."

        objid_to_pubkey = self.update_provers(input)
        self.stage_objects(input, objid_to_pubkey)
        return self.generate_cpm(input, objid_to_pubkey)

    def update_provers(self, input: CPM) -> None:
        # Generate pubkey from proofs and store them.
        # TODO: limit the number of unmatched proofs per sender.
        objid_to_pubkey: Dict[int, Pubkey] = {}
        for objid, p in input.proofs:
            pubkey = pot_pubkey(p, input.sender)

            if pubkey == None:
                # It is not a valid proof.
                continue

            objid_to_pubkey[objid] = pubkey

            if pubkey not in self.pubkey_to_provers:
                self.pubkey_to_provers[pubkey] = set()
            
            self.pubkey_to_provers[pubkey].add(input.sender)
        return objid_to_pubkey

    def stage_objects(self, input: CPM, objid_to_pubkey: Dict[int, Pubkey]) -> None:
        # Stage objects.
        for oid, pos in input.perceived_objects:
            # Only stage objects with valid proof.
            if oid in objid_to_pubkey:
                self._unconfirmed_objects[oid] = pos

    def generate_cpm(self, input: CPM, objid_to_pubkey: Dict[int, Pubkey]) -> CPM:
        # Filter confirmed and unconfirmed objects.
        confirmed = []
        unconfirmed = []

        for oid, pos in self._unconfirmed_objects.items():
            # The Object didn't come with a proof, ignore.
            if oid not in objid_to_pubkey:
                #print("[%s] Warning: dropped object without proof, sender: %s" % (self.vehicle.numberplate, input.sender))
                unconfirmed.append((oid, pos))
                continue

            provers = self.pubkey_to_provers.get(objid_to_pubkey[oid], set())

            if len(provers) < 2:
                unconfirmed.append((oid, pos))
                continue

            confirmed.append((oid, pos))

        self._unconfirmed_objects = dict(unconfirmed)

        return CPM(input.sender, confirmed)

# =============================================================================
#                              Attacker modules
# =============================================================================

class CPSSpammer(VehicleModule):
    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        random = self.vehicle.random
        pos = self.vehicle.position

        objects = [(
            random.randint(0, 10000),
            Position(
                pos.x + random.randint(-100, 100),
                pos.y + random.randint(-100, 100),
                random.random() * 360,
            ),
        ) for i in range(random.randint(0, 32))]

        return CPM(self.vehicle.eid, objects, _gt_is_fake = True)

class CPSReplayer(VehicleModule):
    def do_work(self, input: None) -> CPM:
        assert input == None, f"Module {self.__class__.__name__} requires no input."

        raise NotImplementedError

