#!/usr/bin/python3

import sys
import yaml
from typing import *

from vehicle import *
from simulators import *
from modules import *
from utils import *

def main():
    config = yaml.safe_load(open(sys.argv[1], 'rb'))
    scenario = Scenario(config)

    while True:
        try:
            scenario.tick()
            #scenario.collect_metrics()
        except StopIteration:
            break

    scenario.cleanup()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s config.yaml" % sys.argv[0])
        exit(1)
    main()
