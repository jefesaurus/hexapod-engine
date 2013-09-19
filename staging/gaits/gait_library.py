__author__ = 'glalonde'

import xml.etree.ElementTree as ET
import copy
from staging.gaits.gait import Gait


def parse_gaits():
    tree = ET.parse(parts_path + '/gaits.xml')
    root = tree.getroot()
    for child in root:
        new_gait = Gait.from_xml_node(child)
        gaits[new_gait.type] = new_gait


def get_gait(name):
    if name in gaits:
        return copy.deepcopy(gaits[name])
    raise ValueError('Gait %s not found' % (name))

gaits = {}
parts_path = 'configurations'
parse_gaits()
