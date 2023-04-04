
# This make all type annotations strings (effectively forward declarations.)
from __future__ import annotations

import os
import sys
import math
import random
import numpy as np
import pandas
from typing import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import dataclasses


# Define types.
NumberPlate = str
EID = str
Proof = bytes
Pubkey = bytes

# Define metric types.
VehicleMetric = Dict[NumberPlate, float]
GlobalMetric = float

class MetricCollector:
    def __init__(self, filename: str, collect_function: Callable[[], Union[GlobalMetric, VehicleMetric]], metric_type: str = None):
        self.filename = filename
        self.collect_function = collect_function

        # Check which type of metric is being collected.
        if metric_type is None:
            # Infer metric type from return type of collect_function.
            if collect_function.__annotations__['return'] == GlobalMetric:
                metric_type = 'global'
            elif collect_function.__annotations__['return'] == VehicleMetric:
                metric_type = 'vehicle'
            else:
                raise TypeError('Invalid metric type.')
        self.metric_type = metric_type

        self.data = []

    def collect(self) -> None:
        self.data.append(self.collect_function())

    def save(self):
        if self.metric_type == 'global':
            pandas.DataFrame(self.data, columns=['value']).to_csv(self.filename, columns=['value'], index=False)
        elif self.metric_type == 'vehicle':
            pandas.DataFrame(self.data).to_csv(self.filename, index=False)

# Define Unknown vehicle's number plate.
UNKNOWN_PLATE = 'UNKNOWN'

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
        return (
            int(self.x * (10 ** HASH_ACCURACY_DECIMAL)) * (10 ** HASH_ACCURACY_DECIMAL) +
            int(self.y * (10 ** HASH_ACCURACY_DECIMAL))
        )

    def __repr__(self):
        return f"({self.x}, {self.y})"

@dataclasses.dataclass
class CPM(object):
    sender: EID
    perceived_objects: List[Tuple[int, Position]] = dataclasses.field(default_factory=lambda: [])
    proofs: List[Tuple[int, Proof]] = dataclasses.field(default_factory=lambda: [])
    _objid_to_numberplate: Dict[int, NumberPlate] = dataclasses.field(default_factory=lambda: {})
    _gt_is_fake: bool = False

    def verify(self):
        # Sanity checks to determine whether it is elligible to send on wire.
        assert isinstance(self.sender, EID)
        assert self.sender != ""
        assert self._objid_to_numberplate == {}
        assert len(self.perceived_objects) <= 128
        assert len(self.proofs) <= 8

    def __len__(self):
        # Calculate the total length of the actual CPM packet.

        return (
            34 +                               # 802.11p MAC header.
            4 +                                # GN basic header.
            177 +                              # GN security header.
            68 +                               # GN security trailer.
            8 +                                # GN common header.
            28 +                               # GN SHB header.
            4 +                                # BTP-B header.
            63 +                               # CPM static portion.
            len(self.perceived_objects) * 52 + # Perceived objects.
            len(self.proofs) * 71              # Proofs.
        )

    def __add__(self, other: CPM):
        # Merge two CPMs.

        assert self.sender == other.sender
        assert self._objid_to_numberplate == other._objid_to_numberplate == {}

        # Result is considered fake if any of the two is fake.

        return CPM(
            sender = self.sender,
            perceived_objects = self.perceived_objects + other.perceived_objects,
            proofs = self.proofs + other.proofs,
            _objid_to_numberplate = {},
            _gt_is_fake = self._gt_is_fake or other._gt_is_fake,
        )

def pot_proof(eid: EID, plate: str, salt: EID) -> Proof:
    # XXX: Dummy implementation atm.
    return f"{eid}#{plate}#{salt}".encode()

def pot_pubkey(proof: Proof, salt: EID) -> Pubkey:
    # XXX: Dummy implementation atm.
    eid, plate, salt_ = proof.decode().split('#')

    if salt != salt_:
        return None

    return f"{eid}#{plate}".encode()

