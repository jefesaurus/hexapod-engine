import numpy as np
from kinematics.utils.transform_math import get_unfinished_dh_mat, finish_dh_mat
from parts_library import servo_library
from ik import ik_library
from math import pi # needs to stay for eval
from leg_state import LegState


class Leg:
    def __init__(self, type, a, r, d, servos, servo_offset_angles, servo_orientations, angle_range=None):
        self.type = type
        self.num_joints = len(a)
        self.a = a
        self.r = r
        self.d = d
        self.servos = servos
        self.servo_offset_angles = servo_offset_angles
        self.servo_orientations = servo_orientations
        self.DHP = [None]*self.num_joints
        # Default to servo range: [(min1, max1), (min2, max2)...]
        if angle_range is None:
            self.angle_range = [(servos[i].min_angle_radians, servos[i].max_angle_radians)
                                for i in xrange(self.num_joints)]
        # Default to whatever the most constraining is.
        else:
            self.angle_range = [(max(servos[i].min_angle_radians, angle_range[i][0]),
                                 min(servos[i].max_angle_radians, angle_range[i][1]))
                                for i in xrange(self.num_joints)]
        self.init_computations()
        self.IK = ik_library.get_IK_function(self) #  Grab the appropriate IK function

    # Generate a leg based on an XML node
    @classmethod
    def from_xml_node(cls, node):
        type = node.get('name')
        num_joints = int(node.get('num-joints'))

        a = np.empty(num_joints)
        r = np.empty(num_joints)
        d = np.empty(num_joints)
        servos = [None]*num_joints
        servo_offset_angles = [None]*num_joints
        servo_orientations = [None]*num_joints

        for i in xrange(num_joints):
            segment = node[i]
            a[i] = float(eval(segment.find('a').text))  # Eval'd so, it will handle pi/2 and stuff.
            r[i] = float(eval(segment.find('r').text))
            d[i] = float(eval(segment.find('d').text))
            servo_name = segment.find('servo').text
            servos[i] = servo_library.get_servo(servo_name)
            servo_offset_angles[i] = float(eval(segment.find('servo-offset-angle').text))
            servo_orientations[i] = int(eval(segment.find('servo-orientation').text))
        return cls(type, a, r, d, servos, servo_offset_angles, servo_orientations)

    def init_computations(self):
        (a, r, d) = (self.a, self.r, self.d)
        for i in xrange(self.num_joints):
            self.DHP[i] = get_unfinished_dh_mat(a[i], r[i], d[i])

    def fk_end_point(self, leg_state):
        t = leg_state.servo_angles
        mat = finish_dh_mat(self.DHP[0], t[0])
        for i in xrange(1, self.num_joints):
            mat = mat.dot(finish_dh_mat(self.DHP[i], t[i]))
        mat = mat.dot(np.array([0, 0, 0, 1]))
        return mat[:3]

    def fk_all_points(self, leg_state):
        t = leg_state.servo_angles
        finished_mats = [finish_dh_mat(self.DHP[0], t[0])]
        for i in xrange(1, self.num_joints):
            finished_mats.append(finished_mats[-1].dot(finish_dh_mat(self.DHP[i], t[i])))
        points = [np.array([0, 0, 0])]
        for mat in finished_mats:
            points.append(np.dot(mat, np.array([0, 0, 0, 1]))[:3])
        return points

    def validate_leg_angles(self, leg_angles):
        return [max(min(leg_angles[i], self.angle_range[i][1]), self.angle_range[i][0])
                          for i in xrange(self.num_joints)]
