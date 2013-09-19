__author__ = 'glalonde'
import xml.etree.ElementTree as ET
import copy
from staging.leg import Leg


def parse_legs():
    tree = ET.parse(parts_path + '/legs.xml')
    root = tree.getroot()
    for child in root:
        new_leg = Leg.from_xml_node(child)
        legs[new_leg.type] = new_leg


def get_leg(name):
    if name in legs:
        return copy.deepcopy(legs[name])
    raise ValueError('Leg %s not found'%(name))


legs = {}
parts_path = 'configurations'
parse_legs()