from leg import *
from math import pi,sin,cos


def generateDHMat(t, a, r, d):
  ct = cos(t)
  st = sin(t)
  ca = cos(a)
  sa = sin(a)
  return np.array([[ct,-ca*st, st*sa, r*ct],[st, ct*ca, -ct*sa, r*st],[0, sa, ca, d],[0,0,0,1]])

def generateTransformMat(y, p, r, pos):
    cy = cos(y)#yaw
    sy = sin(y)
    cp = cos(p)#pitch
    sp = sin(p)
    cr = cos(r)#roll
    sr = sin(r)
    rotMat = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
              [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
              [-sp, cp*sr, cp*cr]])
    return np.vstack([np.hstack((rotMat, np.array([pos]).T)), [0,0,0,1]])

class chassis():
  position = [0,0,0]

  yaw = 0
  pitch = 0
  roll = 0
  transformMat = generateTransformMat(yaw, pitch, roll, position)
  inverseTransformMat = np.linalg.inv(transformMat)

  alpha = [ pi/2, 0, 0]
  radius = [.5, 1.5, 2]
  displacement = [0,0,0]

  theta = [[0,   pi/2, -pi/2],
          [ -pi/6,      pi/2, -pi/2],
          [ pi/6,  pi/4, -pi/2],
          [ 0,   pi/4, -pi/2],
          [ -pi/6,      pi/4, -pi/2],
          [ pi/6,  pi/4, -pi/2]]
  print theta


  legThetas = np.array(range(6))*pi/3
  legAlphas = np.array([0]*6)
  legRadii = np.array([1]*6)
  legDisps = np.array([-.5]*6)

  legs = [None]*6
  legDHMats = [None]*6        #Goes from leg frame to chassis frame
  inverseLegDHMats = [None]*6 #Goes from chassis frame to leg frame



  def __init__(self): 
    for i in xrange(len(self.legs)):
      self.legs[i] = leg(self.alpha, self.radius, self.displacement)
      self.legDHMats[i] = generateDHMat(self.legThetas[i],self.legAlphas[i],self.legRadii[i],self.legDisps[i])
      self.inverseLegDHMats[i] = np.linalg.inv(self.legDHMats[i])

  def updateAttitude(self, position, yaw, pitch, roll):
    self.position = position
    self.yaw = yaw
    self.pitch = pitch
    self.roll = roll
    self.transformMat = generateTransformMat(self.yaw, self.pitch, self.roll, self.position)
    self.inverseTransformMat = np.linalg.inv(self.transformMat)

  def getChassisSegments(self):
    segments = []
    for i in xrange(len(self.legs)):
      legPoints = [self.legToEnv(x,i) for x in self.legs[i].FKAllPoints(self.theta[i])]
      segments.append((self.position, legPoints[0]))
      for j in xrange(len(legPoints) - 1):
        segments.append((legPoints[j], legPoints[j+1]))
    return segments
        
  def envToLeg(self, point, leg):
    if len(point) is 3:
      point = np.append(point, [1])
    return np.dot(np.dot(self.inverseLegDHMats[leg],self.inverseTransformMat),point)[:3]

  def legToEnv(self, point, leg):
    if len(point) is 3:
      point = np.append(point, [1])
    return np.dot(np.dot(self.transformMat,self.legDHMats[leg]),point)[:3]

  def getAngles(self, point, leg):
    return self.legs[leg].IK(self.envToLeg(point, leg))
