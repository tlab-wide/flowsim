
# This make all type annotations strings (effectively forward declarations.)
from __future__ import annotations

import os
import sys
import random

if 'SUMO_HOME' in os.environ:
 tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
 sys.path.append(tools)
else:
 sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import dataclasses
import numpy as np


def random_from(parent: Union[random.Random, int]) -> random.Random:
    # Derive a new Random instance from an existing Random or an integer seed.
    if isinstance(parent, random.Random) or parent == random:
        parent = parent.randint(0, 2 ** 32 - 1)
    return random.Random(parent)

# Define type of a vehicle's EID
EID = str

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

    def to_polar(p: Position):
        # Convert point (x, y) to polar coordinates (r, theta).
        return math.hypot(p.x, p.y), math.atan2(self.y, self.x)/pi*180

    def __hash__(self: Position):
        return (int(self.x * (10 ** HASH_ACCURACY_DECIMAL)), 
                int(self.y * (10 ** HASH_ACCURACY_DECIMAL)))

@dataclasses.dataclass
class CPM(object):
    sender: EID
    perceived_objects: List[Tuple[int, Vehicle, Position]] = dataclasses.field(default_factory=lambda: [])
    proofs: List[Proof] = dataclasses.field(default_factory=lambda: [])
    _is_fake: bool = False

    def __len__(self):
        # Calculate the total length of the actual CPM packet.
        raise NotImplementedError

