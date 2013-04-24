import numpy as np
from math import pi,cos,sin

class pose():
  position = (0,0,0)
  yaw = None
  pitch = None
  roll = None

  transformMat = None
  inverseTransformMat = None

  def __init__(self, position, (yaw, pitch, roll)):
    self.position = position
    self.yaw = yaw
    self.pitch = pitch
    self.roll = roll
    cy = cos(yaw)#yaw
    sy = sin(yaw)
    cp = cos(pitch)#pitch
    sp = sin(pitch)
    cr = cos(roll)#roll
    sr = sin(roll)
    rotMat = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
              [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
              [-sp, cp*sr, cp*cr]])
    self.transformMat = np.vstack([np.hstack((rotMat, np.array([position]).T)), [0,0,0,1]])
    self.inverseTransformMat = np.linalg.inv(self.transformMat)

  def toLocal(self, point):
    if len(point) is 3:
      point = np.append(point, [1])
    return np.dot(self.inverseTransformMat,point)[:3]

  def toGlobal(self, point):
    if len(point) is 3:
      point = np.append(point, [1])
    return np.dot(self.transformMat,point)[:3]
