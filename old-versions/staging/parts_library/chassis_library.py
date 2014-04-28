__author__ = 'glalonde'

import xml.etree.ElementTree as ET
import copy
from staging.static_chassis import static_chassis


def parse_chassis():
    tree = ET.parse(parts_path + '/chassis.xml')
    root = tree.getroot()
    for child in root:
        new_chassis = static_chassis.from_xml_node(child)
        chassis[new_chassis.type] = new_chassis


def get_chassis(name):
    if name in chassis:
        return copy.deepcopy(chassis[name])
    raise ValueError('Chassis %s not found'%(name))

chassis = {}
parts_path = 'configurations'
parse_chassis()
