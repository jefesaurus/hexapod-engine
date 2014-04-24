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
  def __init__(self, segments, base_pose):
    self.num_links = len(segments)
    self.segments = segments
    self.base_pose = base_pose
    self.to_frame_mat = base_pose.to_frame_mat
    self.from_frame_mat = base_pose.from_frame_mat
    self.joint_transform_funcs = []
    for i in range(1, self.num_links + 1):
      joint_func, _ = dh_constructor.get_transformation_function(segments[:i], fixed_endpoint=(0,0,0), use_dict=False)
      self.joint_transform_funcs.append(joint_func)

  # Transform a point in the frame of the base_pose, into a global frame
  def to_global(self, angles):
    return self.from_frame_mat.dot(self.joint_transform_funcs[-1](angles))

  # Returns points for each segment in the chain
  def get_all_segments(self, joint_angles):
    for i, func in enumerate(self.joint_transform_funcs):
      yield self.from_frame_mat.dot(func(joint_angles[:i+1]))

def test():
  from pose import Pose
  hip_pose = Pose(5,4,0,pi/2,0,0)
  coxa = Segment('coxa', alpha=pi/2, r=0.5, d=0)
  femur = Segment('femur', alpha=0, r=1.5, d=0)
  tibia = Segment('tibia', alpha=0, r=2, d=0)
  leg = KinematicChain([coxa, femur, tibia], hip_pose)
  pose = (pi/3, -pi/1234, 43*pi)
  for p in leg.get_all_segments(pose):
    print p
  print leg.to_global(pose)

if __name__ == '__main__':
  test()
