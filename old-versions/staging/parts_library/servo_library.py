__author__ = 'glalonde'

import xml.etree.ElementTree as ET
import copy
from staging.servo import servo


def parse_servos():
    tree = ET.parse(parts_path + '/servos.xml')
    root = tree.getroot()
    for child in root:
        new_servo = servo.from_xml_node(child)
        servos[new_servo.type] = new_servo


def get_servo(name):
    if name in servos:
        return copy.deepcopy(servos[name])
    raise ValueError('Servo %s not found'%(name))

servos = {}
parts_path = 'configurations'
parse_servos()
