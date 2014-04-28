__author__ = 'glalonde'
from math import pi, sin, cos, sqrt

class CentralPatternGenerator:
    def __init__(self, chassis, gait):
        self.chassis = chassis
        self.gait = gait
        self.steps_made = [None]*chassis.num_legs

    def get_next_step(self, start_pose, end_pose, ground_z):
        leg_index = 0
        leg_score = self.steps_made[leg_index].l2_norm_squared(end_pose)
        for index in xrange(1, self.chassis.num_legs):
            score = self.steps_made[index].l2_norm_squared(end_pose)
            if score > leg_score:
                leg_index = index
                leg_score = score
        step_point = self._calculate_step(leg_index, start_pose, end_pose, ground_z)
        self.steps_made[leg_index] = end_pose  # Stash the assumption with the leg_index
        return step_point

    def _calculate_step(self, leg_index, start_pose, end_pose, ground_z):
        #get approximate intersection between reachable areas of currentBody pose and desiredBody pose
        #This represents the viable regions for steps for which these poses can be achieved
        #Body pose is ([x,y,z],roll,pitch,yaw)
        leg_pose = self.chassis.leg_poses[leg_index]
        (x_start, y_start, z_start) = start_pose.to_global(leg_pose.position)
        (x_end, y_end, z_end) = end_pose.to_global(leg_pose.position)

        (mid_x, mid_y) = ((x_start + x_end)/2, (y_start + y_end)/2)

        mid_angle = leg_pose.yaw - (start_pose.chassisPose.yaw + end_pose.yaw)/2
        #Use these heights to determine the reachable regions
        (inner_radius_start, outer_radius_start) = self._get_reach_limits(leg_index, z_start, ground_z)
        if z_start is not z_end:
            (inner_radius_end, outer_radius_end) = self._get_reach_limits(leg_index, z_end, ground_z)
        else:
            (inner_radius_end, outer_radius_end) = (inner_radius_start, outer_radius_start)

        #Average all of the limits to get the one furthest from them.
        #This assumes the limits overlap. If they don't, we're hosed.
        target_radius = (inner_radius_start + outer_radius_start + inner_radius_end + outer_radius_end)/4
        step_point = (mid_x + target_radius*sin(mid_angle), mid_y + target_radius*cos(mid_angle), ground_z)
        return step_point

    def _get_reach_limits(self, leg_index, point_z, ground_z):
        leg = self.chassis.legs[leg_index]
        z = (point_z-ground_z)
        z_norm = z**2
        #get min with smallest tibia angle
        min_tibia_angle = max(0, leg.angle_range[2][0]+pi)
        min_foot_femur_norm = leg.r[1]**2+leg.r[2]**2-2*leg.r[1]*leg.r[2]*cos(min_tibia_angle)**2
        if min_foot_femur_norm < z_norm:
            min_radius = 0
        else:
            min_radius = sqrt(min_foot_femur_norm - z_norm)

        max_tibia_angle = min(pi, leg.angle_range[2][1]+pi)
        max_radius = sqrt(leg.r[1]**2+leg.r[2]**2-2*leg.r[1]*leg.r[2]*cos(max_tibia_angle) - z_norm)

        return min_radius, max_radius
