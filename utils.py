
# This make all type annotations strings (effectively forward declarations.)
from __future__ import annotations

import os
import sys
import math
import random
import json
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
        return f"({self.x}, {self.y})"

@dataclasses.dataclass
class Line:
    a: Position
    b: Position
    numberplate: str = None

    def __repr__(self) -> str:
        return f'Line({self.a}, {self.b})'

    def __len__(self):
        return self.a.distance_to(self.b)

    def __hash__(self):
        return hash(self.a) * (10 ** (self.a.HASH_ACCURACY_DECIMAL << 1)) + hash(self.b)

    def get_angle_to_position(self, position: Position):
        return math.fabs((self.a - position).to_polar()[1] - (self.b - position).to_polar()[1])

    def get_subline(self, percent: float = 0.25, center: float = 0.5) -> Line:
        left = max(center - percent / 2.0, 0.0)
        right = min(center + percent / 2.0, 1.0)
        vector = self.a - self.b
        new_lp = Position(self.a.x + vector.x * left, self.a.y + vector.y * left)
        new_rp = Position(self.a.x + vector.x * right, self.a.y + vector.y * right)
        return Line(new_lp, new_rp, self.numberplate)


@dataclasses.dataclass
class Node:
    def __init__(self, angle_start: float = 0.0, angle_end: float = 360.0, data: Line = None,
                 left: 'Node' = None, right: 'Node' = None):
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.data = data
        self.left = left
        self.right = right

    def get_lines(self) -> set[Line]:
        lines = []
        stack = []

        stack.append(self)
        while len(stack) > 0:
            last = stack.pop()
            if last is not None:
                stack.append(last.left)
                stack.append(last.right)
                if last.data is not None and last.data.numberplate is not None:
                    lines.append(last.data)

        return set(lines)

    def __available(self, angle_start: float = 0.0, angle_end: float = 360.0):
        if angle_start > angle_end:
            angle_end, angle_start = angle_start, angle_end

        if angle_end - angle_start > 180.0:
            return self.__available(angle_start=angle_end) and self.__available(angle_end=angle_start)

        if self.left is None and self.right is None:
            return self.angle_start <= angle_start < angle_end <= self.angle_end
        else:
            return self.left.__available(angle_start, angle_end) or self.right.__available(angle_start, angle_end)

    def __update_leftmost(self, data: Line, angle_start: float):
        if self.data is not None:
            self.left.__update_leftmost(data, angle_start)
        else:
            self.data = data
            self.left = Node(angle_end=0.0)
            self.right = Node(angle_start=angle_start, angle_end=self.angle_end)

    def __update_rightmost(self, angle_end: float):
        if self.data is not None:
            self.right.__update_rightmost(angle_end)
        else:
            self.left = Node(angle_start=self.angle_start, angle_end=angle_end)
            self.right = Node(angle_start=360.0)

    def __update_edge(self, data: Line, angle_start: float, angle_end: float):
        self.__update_leftmost(data, angle_start)
        self.__update_rightmost(angle_end)

    def update(self, eye: Position, data: Line = None, angle_start: float = 0.0, angle_end: float = 360.0):
        if data is None:
            return

        if self.left is not None or self.right is not None:
            self.left.update(eye, data)
            self.right.update(eye, data)
            return

        angle_start = eye.angle_to(data.a)
        angle_end = eye.angle_to(data.b)

        if angle_start > angle_end:
            angle_end, angle_start = angle_start, angle_end

        # cross x axis (atan2 == 0)
        if angle_end - angle_start > 180.0 and self.__available(angle_start, angle_end):
            self.__update_edge(data, angle_start, angle_end)
            return

        if self.angle_start <= angle_start < angle_end <= self.angle_end:
            self.data = data
            self.left = Node(self.angle_start, angle_start)
            self.right = Node(angle_end, self.angle_end)
        elif self.angle_start < angle_start < self.angle_end:
            self.angle_end = angle_start
        elif self.angle_end > angle_end > self.angle_start:
            self.angle_start = angle_end


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

