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
  position = None
  yaw = None
  pitch = None
  roll = None
  transformMat = None
  inverseTransformMat = None
  alpha = None
  radius = None 
  displacement = None 
  angleRange = None
  legThetas = None
  legAlphas = None 
  legRadii = None 
  legDisps = None

  legs = [None]*6
  legDHMats = [None]*6        #Goes from leg frame to chassis frame
  inverseLegDHMats = [None]*6 #Goes from chassis frame to leg frame

  def __init__(self, params): 
    #stash params
    self.position = params.position
    self.yaw = params.yaw
    self.pitch = params.pitch
    self.roll = params.roll
    self.alpha = params.alpha
    self.radius = params.radius
    self.displacement = params.displacement
    self.angleRange = params.angleRange
    self.legThetas = params.legThetas
    self.legAlphas = params.legAlphas
    self.legRadii = params.legRadii
    self.legDisps = params.legDisps

    #Initialize one-time calculable stuff
    self.transformMat = generateTransformMat(self.yaw, self.pitch, self.roll, self.position)
    self.inverseTransformMat = np.linalg.inv(self.transformMat)
    for i in xrange(len(self.legs)):
      self.legs[i] = leg(self.alpha[i], self.radius[i], self.displacement[i], self.angleRange[i])
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

  def getFeet(self):
    feet = []
    for i in xrange(len(self.legs)):
      feet.append(self.legToEnv(self.legs[i].FKEndPoint(self.theta[i]),i))
    return feet
    
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
