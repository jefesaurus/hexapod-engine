from math import copysign

from kinematics.pose import *
from chassis_state import ChassisState


class dynamic_model():
    def __init__(self, static_chassis, initial_state=None):
        self.chassis = static_chassis
        if initial_state:
            self.state = initial_state
            self.command = initial_state
        else:
            self.state = ChassisState(static_chassis.home_stance, pose((0, 0, 0), (0, 0, 0)))
            self.command = ChassisState(static_chassis.home_stance, pose((0, 0, 0), (0, 0, 0)))

    # Send commands to the model in the form of a desired pose (all servo angles)
    def set_current_command(self, chassis_state):
        self.command = chassis_state

    # Proceed forward for seconds
    def update_servos(self, time_seconds):
        for (leg, state, command) in zip(self.chassis.legs, self.state.leg_states, self.command.leg_states):
            # Get largest possible movement
            servo_deltas = [leg.servos[i].angular_speed * time_seconds for i in xrange(leg.num_joints)]
            # Truncate to amount necessary to reach destination, if necessary
            servo_deltas = [min((abs(command.servo_angles[i] - state.servo_angles[i])), servo_deltas[i])
                            for i in xrange(leg.num_joints)]
            # Flip sign if necessary
            servo_deltas = [copysign(servo_deltas[i], command.servo_angles[i] - state.servo_angles[i])
                            for i in xrange(leg.num_joints)]
            # Get updated leg state
            potential_leg_state = state.get_updated(servo_deltas)
            # Validate and set
            state.set(leg.validate_leg_angles(potential_leg_state))

    #For use mainly with drawing:
    #Calculates the intermediate positions of all the joints n stuff
    #Extra overhead.
    def get_chassis_segments(self):
        segments = []
        for i in xrange(self.chassis.num_legs):
            leg_points = [self.leg_to_env(x, i) for x in self.chassis.legs[i].fk_all_points(self.state.leg_states[i])]
            segments.append((self.state.chassis_pose.position, leg_points[0]))
            for j in xrange(len(leg_points) - 1):
                segments.append((leg_points[j], leg_points[j+1]))
        return segments

    #update new pose with deltas
    def update_pose(self, (dx, dy, dz), dYaw, dPitch, dRoll):
        new_pose = pose((self.state.chassis_pose.position[0] + dx,
                         self.state.chassis_pose.position[1] + dy,
                         self.state.chassis_pose.position[2] + dz),
                        (self.state.chassis_pose.yaw + dYaw,
                         self.state.chassis_pose.pitch + dPitch,
                         self.state.chassis_pose.roll + dRoll))
        self.state.chassis_pose = new_pose

    #Main FK routine, only returns the feet
    def get_feet(self):
        feet = []
        for i in xrange(self.chassis.num_legs):
            feet.append(self.leg_to_env(self.chassis.legs[i].fk_end_point(self.state.leg_states[i]), i))
        return feet

    #Transforms a point from the global coordinates, to the coordinates of the supplied leg
    #The new coordinates have the coxa joint at 0,0,0, facing down the x axis
    def env_to_leg(self, point, leg):
        return self.state.leg_states[leg].to_local(self.state.chassis_pose.to_local(point))

    #Transforms a point from the leg coordinates as described above into the global coordinates
    def leg_to_env(self, point, leg):
        return self.state.chassis_pose.to_global(self.chassis.leg_poses[leg].to_global(point))

    #Invokes the IK routines for a given leg on global coordinates
    def get_angles(self, point, leg):
        return self.legs[leg].IK(self.env_to_leg(point, leg))
