
# This make all type annotations strings (effectively forward declarations.)
from __future__ import annotations

import os
import sys
import math
import random
import json
import numpy as np
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
        self.file = open(filename, 'w')

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

        self.data = None

    def collect(self) -> None:
        self.data = self.collect_function()
        self.file.write(json.dumps(self.data) + '\n')

    def save(self) -> None:
        # Flush the file.
        self.file.flush()

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

    def __sub__(p1: Position, p2: Position) -> Position:
        # x, y: p2 - p1
        # heading: nan
        return Position(p1.x - p2.x, p1.y - p2.y, np.nan)

    def to_polar(self) -> Tuple[float, float]:
        # Convert point (x, y) to polar coordinates (r, theta).
        # theta is in range [0, 360).
        return (
            math.hypot(self.x, self.y),
            (math.atan2(self.y, self.x) / math.pi * 180 + 360) % 360
        )

    def angle_to(self, other: Position) -> float:
        return (math.atan2(other.y - self.y, other.x - self.x) / math.pi * 180 + 360) % 360
        #return (other - self).to_polar()[1]


    def distance_to(self, other: Position) -> float:
        return math.hypot(other.x - self.x, other.y - self.y)
        #return (other - self).to_polar()[0]


    def __hash__(self: Position):
        return (
            int(self.x * (10 ** self.HASH_ACCURACY_DECIMAL)) * (10 ** self.HASH_ACCURACY_DECIMAL) +
            int(self.y * (10 ** self.HASH_ACCURACY_DECIMAL))
        )

    def __repr__(self):
        return f'Position({self.x:.2f}, {self.y:.2f}, {self.heading:.0f})'

@dataclasses.dataclass
class SegmentTreeNode:
    start: float
    end: float
    left: Optional['SegmentTreeNode'] = None
    right: Optional['SegmentTreeNode'] = None
    covered: bool = False

class SegmentTree:
    def __init__(self, all_points):
        self._sorted_points = sorted(all_points)
        self.mapping = {p: i for i, p in enumerate(self._sorted_points)}
        self.root = SegmentTreeNode(0, len(self._sorted_points) - 1)

    def _insert(self, node, start, end):
        if node is None:
            return SegmentTreeNode(self.mapping[start], self.mapping[end])

        if end <= node.start:
            node.left = self._insert(node.left, start, end)
        elif start >= node.end:
            node.right = self._insert(node.right, start, end)
        else:
            mid = (node.start + node.end) // 2
            node.left = self._insert(node.left, start, self._sorted_points[mid - 1])
            node.right = self._insert(node.right, self._sorted_points[mid], end)

        return node

    def insert(self, start, end):
        self.root = self._insert(self.root, start, end)

    def _query(self, node, start, end):
        if node is None:
            return False

        if start <= node.start and end >= node.end:
            return node.covered
        elif end <= node.start or start >= node.end:
            return False
        else:
            return self._query(node.left, start, end) or self._query(node.right, start, end)

    def query(self, start, end):
        return self._query(self.root, self.mapping[start], self.mapping[end])


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

        assert set(self.perceived_objects) & set(other.perceived_objects) == set()
        assert set(self.proofs) & set(other.proofs) == set()

        return CPM(
            sender = self.sender,
            perceived_objects = self.perceived_objects + other.perceived_objects,
            proofs = self.proofs + other.proofs,
            _objid_to_numberplate = {},
            # Result is considered fake if any of the two is fake.
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

class RingBuffer:
    # A fixed-size ring buffer.
    def __init__(self, size: int, default_factory: Callable[[], Any] = lambda: None):
        self.size = size
        self.default_factory = default_factory
        self.data = [default_factory() for _ in range(size)]

    @property
    def current(self):
        return self.data[0]

    @current.setter
    def current(self, value: Any):
        self.data[0] = value

    def advance(self, new_data: Any = None):
        if new_data == None:
            new_data = self.default_factory()
        self.data = [new_data] + self.data[:-1]

