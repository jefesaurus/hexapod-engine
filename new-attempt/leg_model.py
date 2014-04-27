import kinematic_chain as kc
from math import pi, atan, acos, sqrt, atan2
import numpy as np

# A revolute joint with state about the angle, range, speed etc.
class LegSegment(object):
  def __init__(self, revolute_joint):
    #self.servo = servo
    self.current_angle = 0
    self.revolute_joint = revolute_joint

  def set_angle(self, angle):
    self.current_angle = angle

  def update(self, commanded_angle, time_elapsed):
    pass

class Leg(object):
  def __init__(self, leg_segments, ik_func):
    self.frame = kc.KinematicChain([seg.revolute_joint for seg in leg_segments])
    self.leg_segments = leg_segments
    self.ik_func = ik_func
    
  def get_ik_solution(self, x, y, z):
    return self.ik_func(self, x, y, z)

  def set_angles(self, angles):
    for seg, angle in zip(self.leg_segments, angles):
      seg.current_angle = angle

  # Commanded Point must be in the legs frame.
  def update(self, commanded_point, time_elapsed):
    # Get req'd angles out of IK
    # update
    pass

  def get_segments(self):
    angles = [segment.current_angle for segment in self.leg_segments]
    return self.frame.get_all_segments(angles)

  def get_end_effector(self):
    angles = [segment.current_angle for segment in self.leg_segments]
    return self.frame.to_global(angles)


# Inverse Kinematics for a very specific 3DoF leg structure
def IK_3DoF(kinematic_chain, x, y, z):
  coxa, femur, tibia = kinematic_chain.kinematic_pairs
  x, y, z = float(x), float(y), float(z)

  # The combined offset along the axes of rotation for the femur and tibia
  d = femur.d + tibia.d
  # There is only one possible coxa angle for any point
  det = x**2 + y**2 - d**2
  if det < 0:
    return (0, 0, 0)
  elif d == y: # Some weird degenerate case. Couldn't quite figure out why, but this gets around it
    coxa_angle = 2*atan(y/x)
  else:
    coxa_angle = 2*atan((x - sqrt(x**2 + y**2 - d**2))/(d - y))

  # Now move into the coxa's frame using the calculated angle
  nx, ny, nz, _ = kinematic_chain.inv_transform_funcs[0]((x,y,z), coxa_angle).T[0]

  # Now we only care about X and Y. Z should equal d
  target = nx**2+ny**2
  femur.r = femur.r
  tibia.r = tibia.r
  range = (femur.r + tibia.r)**2  
  target_dir = atan2(ny, nx)
  if target > range:      # Too far
    solutions = (coxa_angle, target_dir, 0) 
  elif target < (femur.r - tibia.r)**2: # Too close
    solutions = (coxa_angle, 0, 0)
  else:
    # law of cosines
    theta_a = acos((femur.r**2 + tibia.r**2 - target)/(2*femur.r * tibia.r))  # The 'inner tibia angle'
    theta_b = acos((femur.r**2 + target - tibia.r**2)/(2*femur.r * sqrt(target)))

    # TODO use servo range limits to select between the two solutions
    solutions = [(coxa_angle, target_dir + theta_b, theta_a - pi), (coxa_angle, target_dir - theta_b, pi - theta_a)]
  return solutions

def get_test_leg():
  coxa = LegSegment(kc.RevoluteJoint('coxa', pi/2, 0.5, 0))
  femur = LegSegment(kc.RevoluteJoint('femur', 0, 1.5, .1))
  tibia = LegSegment(kc.RevoluteJoint('tibia', 0, 2, .1))
  return Leg([coxa, femur, tibia], IK_3DoF)

def test():
  leg = get_test_leg()
  angles = IK_3DoF(leg.frame, 3.145, -1, -2)
  print angles
  leg.set_angles(angles[0])
  print leg.get_end_effector()

if __name__ == '__main__':
  test()
