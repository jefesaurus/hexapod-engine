import kinematic_chain as kc
import time
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

  def set_command(self, joint_command):
    self.destination = joint_command[0]
    self.angular_velocity = joint_command[1]

class LegModel(object):
  def __init__(self, revolute_joints):
    self.frame = kc.KinematicChain(revolute_joints)
    self.joint_states = [RevoluteState(joint) for joint in revolute_joints]

  def set_angles(self, angles):
    for state, angle in zip(self.joint_states, angles):
      state.set_theta(angle)

  def update_state(self, time_elapsed):
    for joint in self.joint_states:
      joint.update_state(time_elapsed)

  def set_joint_commands(self, joint_commands): # Commanded point must be in leg's frame
    for (joint, joint_command) in zip(self.joint_states, joint_commands):
      joint.set_command(joint_command)

  def get_segments(self):
    angles = [joint.current_theta for joint in self.joint_states]
    return self.frame.get_all_segments(angles)

  def get_end_effector(self):
    angles = [segment.current_theta for segment in self.joint_states]
    return self.frame.to_global(angles)


UPDATE_INTERVAL = .02
# Input: [(angle, velocity), ...]
# Output: segments
def leg_model_updater(leg_model, servo_command_input, segment_output):
  last_time = time.time()
  while True:
    start_time = time.time()
    command = None
    while servo_command_input.poll():
      command = servo_command_input.recv()
      if command == 'KILL':
        segment_output.send('KILL')
        return
    if command is not None:
      leg_model.set_joint_commands(command)
    current_time = time.time()
    leg_model.update_state(current_time - last_time)
    last_time = current_time
    segment_output.send(leg_model.get_segments())
    time.sleep(UPDATE_INTERVAL - (time.time() - start_time))

def get_test_leg():
  coxa = kc.RevoluteJoint('coxa', math.pi/2, 0.25, 0, max_theta = math.pi/3, min_theta=-math.pi/3)
  femur = kc.RevoluteJoint('femur', 0, 1.5, 0, min_theta=-math.pi/2, max_theta=math.pi/2)
  tibia = kc.RevoluteJoint('tibia', 0, 2.5, 0, min_theta=-math.pi, max_theta=0)
  return LegModel([coxa, femur, tibia])

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
