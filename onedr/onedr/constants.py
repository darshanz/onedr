# Some ArduPilot modes only for now
from enum import Enum

class Mode(Enum):
    STABILIZE = 0
    GUIDED = 4
    LOITER = 5
    RTL = 6
    AUTO = 3
    LAND = 9

class ArmState():
    DISARM = 0
    ARM = 1
