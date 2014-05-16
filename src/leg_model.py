import kinematic_chain as kc
from ik_3dof import IK_3DoF

import math

# A revolute joint with state about the angle, range, speed etc.
class JointState(object):
  def __init__(self, kinematic_pair):
    self.kp = kinematic_pair
    if kinematic_pair.alpha is None:
      self.current_alpha = 0
    if kinematic_pair.r is None:
      self.current_r = 0
    if kinematic_pair.d is None:
      self.current_d = 0
    if kinematic_pair.theta is None:
      self.current_theta = 0


class RevoluteState(JointState):
  def __init__(self, revolute_joint):
    super(RevoluteState, self).__init__(revolute_joint)
    self.destination = None
    self.angular_velocity = 1.

  def set_theta(self, new_theta):
    if new_theta > self.kp.max_theta:
      self.current_theta = self.kp.max_theta
    elif new_theta < self.kp.min_theta:
      self.current_theta = self.kp.min_theta
    else:
      self.current_theta = new_theta

  def update_state(self, time_elapsed):
    if self.destination is not None:
      diff = self.destination - self.current_theta
      delta = math.copysign(self.angular_velocity*time_elapsed, diff)
      if abs(delta) < abs(diff):
        self.set_theta(self.current_theta + delta)
      else:
        self.set_theta(self.destination)

  def set_command(self, destination, angular_velocity):
    self.destination = destination
    self.angular_velocity = angular_velocity

STEP_HEIGHT = 1
OVERSHOOT_PERCENT = .5
ESTIMATE_REFRESH_RATE=.05

MAX_ANGULAR_VELOCITY = 1.
class LegModel(object):
  def __init__(self, revolute_joints, ik_func):
    self.frame = kc.KinematicChain(revolute_joints)
    self.joint_states = [RevoluteState(joint) for joint in revolute_joints]
    self.ik_func = ik_func

    # For interpolation
    self.deadline = 0
    self.current_time = 0
    self.interval_estimate = ESTIMATE_REFRESH_RATE
    self.p0 = None
    self.p1 = None
    self.p2 = None
    self.p3 = None
    
  def get_ik_solution(self, x, y, z):
    return self.ik_func(self, self.frame, x, y, z)

  def set_angles(self, angles):
    for state, angle in zip(self.joint_states, angles):
      state.set_theta(angle)

  def update_leg(self, time_elapsed):
    for joint in self.joint_states:
      joint.update_state(time_elapsed)

  def set_joint_commands(self, x, y, z): # Commanded point must be in leg's frame
    ik_solutions = self.ik_func(self.frame, x, y, z)
    if len(ik_solutions) > 0:
      pairs = zip(self.joint_states, ik_solutions[0])
      # Figure out which one will take the longest, and scale all of speeds
      # so they finish at the same time.
      completion_times = [abs(joint.current_theta - angle)/MAX_ANGULAR_VELOCITY for joint, angle in pairs]
      max_time = max(max(completion_times), .00000001)
      #if max_time > self.interval_estimate:
      #  print 'TOO FAST: ' + str(max_time) + '\n'

      for (joint, angle), time in zip(pairs, completion_times):
        joint.set_command(angle, MAX_ANGULAR_VELOCITY*time/max_time)

  def get_segments(self):
    angles = [joint.current_theta for joint in self.joint_states]
    return self.frame.get_all_segments(angles)

  def get_end_effector(self):
    angles = [segment.current_theta for segment in self.joint_states]
    return self.frame.to_global(angles)

  def set_command(self, x, y, z, deadline):
    self.deadline = deadline
    self.current_time = 0
    cx, cy, cz,_ = self.get_end_effector()
    self.p0 = (cx, cy, cz)
    self.p1 = (cx, cy, cz + STEP_HEIGHT)
    self.p2 = (x, y, z + STEP_HEIGHT)
    self.p3 = (x,y,z)
    #print str(self.p0) + ' to ' + str(self.p3) + ' in: ' + str(self.deadline) + '\n'

  def update_state(self, time_elapsed):
    self.current_time += time_elapsed
    self.interval_estimate = .5*self.interval_estimate + .5*time_elapsed
    if self.deadline > 0 and self.p0 is not None:
      dest_point = min((self.current_time + self.interval_estimate)/self.deadline, 1)
      #print str(dest_point) + '\n'
      if dest_point < 1:
        x = cubic_bezier(dest_point, self.p0[0], self.p1[0], self.p2[0], self.p3[0])
        y = cubic_bezier(dest_point, self.p0[1], self.p1[1], self.p2[1], self.p3[1])
        z = cubic_bezier(dest_point, self.p0[2], self.p1[2], self.p2[2], self.p3[2])
        self.set_joint_commands(x, y, z)
      else:
        self.set_joint_commands(self.p3[0], self.p3[1], self.p3[2])
    self.update_leg(time_elapsed)

def cubic_bezier(t, p0, p1, p2, p3):
  return ((1-t)**3)*p0 + ((1-t)**2)*3*t*p1 + (t**2)*3*(1-t)*p2 + (t**3)*p3


    
def get_test_leg():
  coxa = kc.RevoluteJoint('coxa', math.pi/2, 0.25, 0, max_theta = math.pi/3, min_theta=-math.pi/3)
  femur = kc.RevoluteJoint('femur', 0, 1.5, 0, min_theta=-math.pi/2, max_theta=math.pi/2)
  tibia = kc.RevoluteJoint('tibia', 0, 2.5, 0, min_theta=-math.pi, max_theta=0)
  return LegModel([coxa, femur, tibia], IK_3DoF)

def test():
  leg = get_test_leg()
  angles = IK_3DoF(leg.frame, 3.145, -1, -2)
  print angles
  leg.set_angles(angles[0])
  print leg.get_end_effector()

def ik_speed_test():
  import random
  import time
  domain = 1.5
  count = 6000

  leg = get_test_leg()
  start = time.time()
  for i in range(count):
    x = random.random()*domain + 2
    y = random.random()*domain
    z = random.random()*domain - 1.
    IK_3DoF(leg.frame, x, y, z)
  print "Time for %d iterations: < %s seconds"%(count,str(time.time() - start))

if __name__ == '__main__':
  ik_speed_test()
