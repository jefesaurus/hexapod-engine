__author__ = 'glalonde'

from leg_state import LegState
from kinematics.pose import pose


class ChassisState:
    def __init__(self, leg_states, chassis_pose):
        self.num_legs = len(leg_states)
        self.leg_states = leg_states
        self.chassis_pose = chassis_pose

    @classmethod
    def get_random(cls, num_legs):
        return cls([LegState.get_random(3) for i in xrange(num_legs)], pose((0, 0, 0), (0, 0, 0)))

    # TODO from-xml
