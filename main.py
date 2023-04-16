#!/usr/bin/env python3

import sys
import yaml

import simulators

def main():
    config = yaml.safe_load(open(sys.argv[1], 'rb'))
    scenario = simulators.Scenario(config)

    while True:
        try:
            scenario.tick()
        except StopIteration:
            break

    scenario.cleanup()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s config.yaml" % sys.argv[0])
        exit(1)
    main()
