__author__ = 'glalonde'


class LegState:
    def __init__(self, servo_angles):
        self.num_joints = len(servo_angles)
        self.servo_angles = servo_angles

    @classmethod
    def get_random(cls, num_joints):
        import random
        from math import pi
        return cls([random.random()*pi/2-pi/4 for i in xrange(num_joints)])

    def get_updated(self, servo_deltas):
        return [self.servo_angles[i] + servo_deltas[i] for i in xrange(self.num_joints)]

    def set(self, angles):
        self.servo_angles = angles
