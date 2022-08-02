'''
Store the fixed locations of the DWM1001-Dev devices configured as anchors
'''
from collections import namedtuple

Location = namedtuple('Location', ['x', 'y'])

ANCHOR_LOCATIONS = [Location(0, 0), Location(2.4, 0), Location(2.5, 2.6),
        Location(-0.1, 2.6)]

