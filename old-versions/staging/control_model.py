__author__ = 'glalonde'

from kinematics.pose import pose

class controller_model:
    def __init__(self, static_chassis):
        self.chassis = static_chassis
        self.chassis_pose = pose((0, 0, 0), (0, 0, 0))

    def set_foot_positions(self, foot_positions):
        self.foot_positions = self.chassis_pose.to_global(foot_positions)

    def get_leg_states(self):
        return [self.chassis.legs[i].IK(self.foot_positions[i]) for i in xrange(self.chassis.num_legs)]
