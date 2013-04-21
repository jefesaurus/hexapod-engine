from leg import *
from math import pi



class chassis():
  position = [0,0,0]

  yaw = 0
  cosYaw = None
  sinYaw = None
  pitch = 0 
  cosPitch = None
  sinPitch = None
  roll = 0 
  cosRoll = None
  sinRoll = None
  transformMat = None

  alpha = [0, pi/2, 0, 0]
  radius = [1., .5, 1.5, 2]
  displacement = [-.5,0,0,0]

  theta = [[0, 0, pi/4, -pi/2],
          [pi/3, 0, pi/4, -pi/2],
          [2*pi/3, 0, pi/4, -pi/2],
          [pi, 0, pi/4, -pi/2],
          [4*pi/3, 0, pi/4, -pi/2],
          [5*pi/3, 0, pi/4, -pi/2]]


  legs = [None]*6


  def __init__(self, parameters=None): 
    if parameters is None:
      for i in xrange(len(self.theta)):
        self.legs[i] = leg(self.theta[i], self.alpha, self.radius, self.displacement)
    else:
      for i in xrange(len(parameters[0])):
        self.legs[i] = leg(parameters[0][i], parameters[1], parameters[2], parameters[3])
    self.updateTrig()

  def updateAttitude(self, position, yaw, pitch, roll):
    self.position = position
    self.yaw = yaw
    self.pitch = pitch
    self.roll = roll
    self.updateTrig()

  def updateTrig(self):
    self.cosYaw = np.cos(self.yaw)
    self.sinYaw = np.sin(self.yaw)
    self.cosPitch = np.cos(self.pitch)
    self.sinPitch = np.sin(self.pitch)
    self.cosRoll = np.cos(self.roll)
    self.sinRoll = np.sin(self.roll)
    self.transformMat = np.array([[self.cosYaw*self.cosPitch, self.cosYaw*self.sinPitch*self.sinRoll - self.sinYaw*self.cosRoll, self.cosYaw*self.sinPitch*self.cosRoll + self.sinYaw*self.sinRoll, self.position[0]],
                                  [self.sinYaw*self.cosPitch, self.sinYaw*self.sinPitch*self.sinRoll + self.cosYaw*self.cosRoll, self.sinYaw*self.sinPitch*self.cosRoll - self.cosYaw*self.sinRoll, self.position[1]],
                                  [-self.sinPitch, self.cosPitch*self.sinRoll, self.cosPitch*self.cosRoll, self.position[2]]])

  def getChassisSegments(self):
    segments = []
    for i in xrange(len(self.legs)):
      legPoints = [self.applyChassisTransform(x) for x in self.legs[i].getAllJoints(self.theta[i])]
      segments.append((self.position, legPoints[0]))
      for j in xrange(len(legPoints) - 1):
        segments.append((legPoints[j], legPoints[j+1]))
    return segments
        
  def applyChassisTransform(self, point):
    if len(point) is 3:
      point = np.append(point, [1])
    return np.dot(self.transformMat, point)[:3]
    

    
  




      

