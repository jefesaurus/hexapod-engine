from math import pi
from leg_state import LegState
from kinematics.pose import pose
from parts_library import leg_library
import glob
import xml.etree.ElementTree as ET


class static_chassis:
    def __init__(self, type, legs, leg_poses, home_stance):
        self.type = type
        self.legs = legs
        self.num_legs = len(legs)
        self.leg_poses = leg_poses
        self.home_stance = home_stance

    # Generate a chassis based on an XML node
    @classmethod
    def from_xml_node(cls, node):
        type = node.get('name')
        num_legs = int(node.get('num-legs'))
        legs = []
        leg_poses = []
        home_state = []

        for i in xrange(num_legs):
            current_leg = node[i]
            leg_frame = leg_library.get_leg(current_leg.get('name'))
            leg_pose = pose(eval(current_leg.get('position')), eval(current_leg.get('orientation')))
            leg_home_state = LegState(eval(current_leg.get('home', None)))

            legs.append(leg_frame)
            leg_poses.append(leg_pose)
            home_state.append(leg_home_state)

        return cls(type, legs, leg_poses, home_state)

    def load_config(self, path):
        if path[-1] is not '/':
            directory = path + '/'
        else:
            directory = path

        for file_ in glob.glob(directory + '*.xml'):
            tree = ET.parse(file_)
            root = tree.getroot()
            for child in root:
                print child.tag, child.attrib
