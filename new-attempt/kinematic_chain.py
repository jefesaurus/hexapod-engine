import fk_math
from math import pi

# Represents a single link in the Denavit Hartenberg representation of a robotic manipulator
class KinematicPair(object):
  def __init__(self, label, alpha=None, r=None, d=None, theta=None):
    self.label = label
    self.alpha = alpha
    self.r = r
    self.d = d
    self.theta = theta

class RevoluteJoint(KinematicPair):
  def __init__(self, label, alpha, r, d):
    super(RevoluteJoint, self).__init__(label, alpha=alpha, r=r, d=d)

class PrismaticJoint(KinematicPair):
  def __init__(self, label, alpha, d, theta):
    super(PrismaticJoint, self).__init__(label, alpha=alpha, d=d, theta=theta)

class CylindricalJoint(KinematicPair):
  def __init__(self, label, alpha, d):
    super(CylindricalJoint, self).__init__(label, alpha=alpha, d=d)

class KinematicChain(object):
  def __init__(self, kinematic_pairs):
    self.num_links = len(kinematic_pairs)
    self.kinematic_pairs = kinematic_pairs
    self.transform_funcs = []
    self.inv_transform_funcs = []
    for i in range(1, self.num_links + 1):
      transform_func, inv_transform_func = fk_math.get_transformation_function(kinematic_pairs[:i], fixed_endpoint=(0,0,0), use_dict=False)
      self.transform_funcs.append(transform_func)
      self.inv_transform_funcs.append(inv_transform_func)

  def to_global(self, var_vals):
    return self.transform_funcs[-1](var_vals)

  # Returns points for each segment in the chain
  def get_all_segments(self, var_vals):
    points = [[0,0,0]]
    for i, func in enumerate(self.transform_funcs):
      points.append(func(var_vals[:i+1])[:3].T[0].tolist())
    return [(points[x-1], points[x]) for x in xrange(1,len(points))]

def test():
  coxa = RevoluteJoint('coxa', pi/2, 0.5, 0)
  femur = RevoluteJoint('femur', 0, 1.5, 0)
  tibia = RevoluteJoint('tibia', 0, 2, 0)
  leg = KinematicChain([coxa, femur, tibia])
  pose = (pi/4, 0, 0)
  for p in leg.get_all_segments(pose):
    print p
  print leg.to_global(pose)

if __name__ == '__main__':
  test()
