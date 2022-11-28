
# This make all type annotations strings (effectively forward declarations.)
from __future__ import annotations

import os
import sys
import math
import random
from typing import *

if 'SUMO_HOME' in os.environ:
 tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
 sys.path.append(tools)
else:
 sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import dataclasses
import numpy as np


# Define types.
NumberPlate = str
EID = str
Proof = bytes
Pubkey = bytes

# Define metric types.
VehicleMetric = Dict[NumberPlate, int]
GlobalMetric = int

def random_from(parent: Union[random.Random, int]) -> random.Random:
    # Derive a new Random instance from an existing Random or an integer seed.
    if isinstance(parent, random.Random) or parent == random:
        parent = parent.randint(0, 2 ** 32 - 1)
    return random.Random(parent)

@dataclasses.dataclass
class Position(object):
    x: float
    y: float
    heading: float = np.nan
    HASH_ACCURACY_DECIMAL: int = 2

    def __sub__(p1: Position, p2: Position):
        # x, y: p2 - p1
        # heading: nan
        return Position(p1.x - p2.x, p1.y - p2.y, np.nan)

    def to_polar(self):
        # Convert point (x, y) to polar coordinates (r, theta).
        # theta is in range [0, 360).
        return (
            math.hypot(self.x, self.y),
            (math.atan2(self.y, self.x)/math.pi * 180 + 360) % 360
        )

    def __hash__(self: Position):
        return (int(self.x * (10 ** HASH_ACCURACY_DECIMAL)), 
                int(self.y * (10 ** HASH_ACCURACY_DECIMAL)))

@dataclasses.dataclass
class CPM(object):
    sender: EID
    perceived_objects: List[Tuple[int, Vehicle, Position]] = dataclasses.field(default_factory=lambda: [])
    proofs: List[Tuple[int, Proof]] = dataclasses.field(default_factory=lambda: [])
    _is_fake: bool = False

    def verify(self):
        # Sanity checks to determine whether it is elligible to send on wire.
        assert isinstance(self.sender, EID)
        assert self.sender != ""
        assert len(self.perceived_objects) <= 128
        assert len(self.proofs) <= 8

    def __len__(self):
        # Calculate the total length of the actual CPM packet.
        raise NotImplementedError


def pot_proof(eid: EID, plate: str, salt: EID) -> Proof:
    # XXX: Dummy implementation atm.
    return f"{eid}#{plate}#{salt}".encode()

def pot_pubkey(proof: Proof) -> Pubkey:
    # XXX: Dummy implementation atm.
    eid, plate, salt = proof.decode().split('#')
    return f"{eid}#{plate}".encode()

