import numpy as np
from math import cos, sin, pi

class Pose:
  def __init__(self, x, y, z, yaw, pitch, roll):
    self.x = x
    self.y = y
    self.z = z
    self.yaw = yaw
    self.pitch = pitch
    self.roll = roll
    cy = cos(yaw)  # yaw
    sy = sin(yaw)
    cp = cos(pitch)  # pitch
    sp = sin(pitch)
    cr = cos(roll)  # roll
    sr = sin(roll)
    self.from_frame_mat = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x],
                          [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y],
                          [-sp, cp*sr, cp*cr, z],
                          [0,0,0,1]])
    self.to_frame_mat = np.linalg.inv(self.from_frame_mat)

  def get_transforms_as_list(self):
    return self.from_frame_mat.tolist(), self.to_frame_mat.tolist()

  def from_frame_c(self, x, y, z):
    return self.from_frame_mat.dot(np.array([x,y,z,1]))[:3]

  def to_frame_c(self, x, y, z):
    return self.to_frame_mat.dot(np.array([x,y,z,1]))[:3]

  def as_tuple(self):
    return (self.x, self.y, self.z, self.yaw, self.pitch, self.roll)
  
  @classmethod
  def from_tuple(cls, pt):
    return cls(pt[0], pt[1], pt[2], pt[3], pt[4], pt[5])


def test():
  import random
  import time
  domain = 10
  count = 50000
  max_wrong = 0.
  start = time.time()
  for i in range(count):
    x = random.random()*domain - domain/2.
    y = random.random()*domain - domain/2.
    z = random.random()*domain - domain/2.
    px = random.random()*domain - domain/2.
    py = random.random()*domain - domain/2.
    pz = random.random()*domain - domain/2.
    yaw = random.random()*domain - domain/2.
    pitch = random.random()*domain - domain/2.
    roll = random.random()*domain - domain/2.
    pose1 = Pose(x,y,z,yaw,pitch,roll)
    ix, iy, iz = pose1.from_frame_c(px,py,pz)
    nx, ny, nz = pose1.to_frame_c(ix, iy, iz)
    max_wrong = max(max_wrong, abs(px - nx), abs(py - ny), abs(pz - nz))
  print "Time for %d iterations: < %s seconds"%(count*2,str(time.time() - start))
  print "Largest error: " + str(max_wrong)

if __name__ == '__main__':
  test()
