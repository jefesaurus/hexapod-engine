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
  def __init__(self, label, alpha, r, d, min_theta=-pi, max_theta=pi):
    super(RevoluteJoint, self).__init__(label, alpha=alpha, r=r, d=d)
    self.min_theta = min_theta
    self.max_theta = max_theta

class PrismaticJoint(KinematicPair):
  def __init__(self, label, alpha, d, theta, min_r=0, max_r=None):
    super(PrismaticJoint, self).__init__(label, alpha=alpha, d=d, theta=theta)
    self.min_r = min_r
    self.max_r = max_r

class CylindricalJoint(KinematicPair):
  def __init__(self, label, alpha, d, min_r=0, max_r=None, min_theta=-pi, max_theta=pi):
    super(CylindricalJoint, self).__init__(label, alpha=alpha, d=d)
    self.min_r = min_r
    self.max_r = max_r
    self.min_theta = min_theta
    self.max_theta = max_theta

class KinematicChain(object):
  def __init__(self, kinematic_pairs, ik_func=None):
    self.num_links = len(kinematic_pairs)
    self.kinematic_pairs = kinematic_pairs
    self.transform_funcs = []
    self.inv_transform_funcs = []
    for i in range(1, self.num_links + 1):
      transform_func, inv_transform_func = fk_math.get_transformation_function(kinematic_pairs[:i], fixed_endpoint=(0,0,0), use_dict=False)
      self.transform_funcs.append(transform_func)
      self.inv_transform_funcs.append(inv_transform_func)

  def to_global(self, var_vals):
    return self.transform_funcs[-1](var_vals)[:4].T[0].tolist()

  # Returns points for each segment in the chain
  def get_all_segments(self, var_vals):
    points = [[0,0,0,1.]]
    for i, func in enumerate(self.transform_funcs):
      points.append(func(var_vals[:i+1])[:4].T[0].tolist())
    return [(points[x-1], points[x]) for x in xrange(1,len(points))]

  def generate_transformation_code(self, label, dest_path):
    fk_math.generate_transformation_code(self.kinematic_pairs, label, dest_path, fixed_endpoint=(0,0,0), intermediate_points=True)


def test():
  coxa = RevoluteJoint('coxa', pi/2, 0.5, 0)
  femur = RevoluteJoint('femur', 0, 1.5, 0)
  tibia = RevoluteJoint('tibia', 0, 2, 0)
  leg = KinematicChain([coxa, femur, tibia])
  pose = (pi/4, 0, 0)
  for p in leg.get_all_segments(pose):
    print p
  print leg.to_global(pose)
  leg.generate_transformation_code('leg', './gen/basic_leg_transforms')

if __name__ == '__main__':
  test()
