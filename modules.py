
import dataclasses
import abc

from utils import *

class VehicleModule(metaclass=abc.ABCMeta):
    # A module in a Vehicle 

    def __init__(self, vehicle):
        self.vehicle = vehicle

    @abc.abstractmethod
    def do_work(self, input: CPM) -> CPM:
        return input


class LocalPerception(VehicleModule):
    # Local perception module

    pass

class Planner(VehicleModule):
    # Planning module
    pass

class CPSReceiver(VehicleModule):

    def do_work(self, input: CPM) -> CPM:
        return self.vehicle.scenario.network.receive(self.vehicle)
        

class CPSSender(VehicleModule):

    def do_work(self, input: CPM) -> CPM:
        self.vehicle.scenario.network.broadcast(self.vehicle, CPM)
        return None

class PoTProver(VehicleModule):
    pass

class PoTVerifier(VehicleModule):
    pass

