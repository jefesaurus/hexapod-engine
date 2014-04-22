import dh_constructor
from math import pi

# Represents a single link in the Denavit Hartenberg representation of a robotic manipulator
class Segment:
  def __init__(self, label, alpha=None, r=None, d=None, theta=None):
    self.label = label
    self.alpha = alpha
    self.r = r
    self.d = d
    self.theta = theta

class KinematicChain:
  def __init__(self, segments):
    self.num_links = len(segments)
    self.segments = segments
    self.joint_transform_funcs = []
    for i in range(1, self.num_links + 1):
      joint_func, _ = dh_constructor.get_transformation_function(segments[:i], fixed_endpoint=(0,0,0), use_dict=False)
      self.joint_transform_funcs.append(joint_func)
    self.to_global = self.joint_transform_funcs[-1]

  # Returns points for each segment in the chain
  def get_all_segments(self, joint_angles):
    for i, func in enumerate(self.joint_transform_funcs):
      yield func(joint_angles[:i+1])

def test():
  coxa = Segment('coxa', alpha=pi/2, r=0.5, d=0)
  femur = Segment('femur', alpha=0, r=1.5, d=0)
  tibia = Segment('tibia', alpha=0, r=2, d=0)
  leg = KinematicChain([coxa, femur, tibia])
  pose = (pi/4, 0, 0)
  for point in leg.get_all_segments(pose):
    print point
  print leg.to_global(pose)

  

if __name__ == '__main__':
  test()
