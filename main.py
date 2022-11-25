#!/usr/bin/python3

import dataclasses
import random
import yaml
import math
import numpy as np
import pandas
from typing import *

from vehicle import *
from simulators import *
from modules import *
from utils import *


def main():
    config = yaml.safe_load(open(sys.argv[1], 'rb'))
    scenario = Scenario(config)

    for tick in range(scenario.config['total_ticks']):
        scenario.tick()
        scenario.collect_metrics()

