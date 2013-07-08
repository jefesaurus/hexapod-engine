from leg import *
from math import pi,sin,cos
from pose import *


class chassis():
  chassisPose = None

  #poses of legs(hips, really) relative to chassis
  legPose = None

  #actual leg objects containing all of the parameters therein
  legs = None

  #Get an initial pose, the relative positions of the legs, and the leg constructs themselves
  def __init__(self, params): 
    self.chassisPose = params.chassisPose
    self.legPose = params.legPose
    self.legs = params.legs

  #For use mainly with drawing:
  #Calculates the intermediate positions of all the joints n stuff
  #Extra overhead.
  def getChassisSegments(self, theta):
    segments = []
    for i in xrange(len(self.legs)):
      legPoints = [self.legToEnv(x,i) for x in self.legs[i].FKAllPoints(theta[i])]
      segments.append((self.chassisPose.position, legPoints[0]))
      for j in xrange(len(legPoints) - 1):
        segments.append((legPoints[j], legPoints[j+1]))
    return segments

  #update new pose with deltas
  def updatePose(self, (dX,dY,dZ),dYaw,dPitch,dRoll):
    newPose = pose((self.chassisPose.position[0]+dX,self.chassisPose.position[1] + dY, self.chassisPose.position[2] + dZ),(self.chassisPose.yaw+dYaw, self.chassisPose.pitch + dPitch, self.chassisPose.roll+dRoll))
    self.chassisPose = newPose

  #Main FK routine, only returns the feet
  def getFeet(self, theta):
    feet = []
    for i in xrange(len(self.legs)):
      feet.append(self.legToEnv(self.legs[i].FKEndPoint(theta[i]),i))
    return feet
    
  #Transforms a point from the global coordinates, to the coordinates of the supplied leg
  #The new coordinates have the coxa joint at 0,0,0, facing down the x axis
  def envToLeg(self, point, leg):
    return self.legPose[leg].toLocal(self.chassisPose.toLocal(point))

  #Transforms a point from the leg coordinates as described above into the global coordinates
  def legToEnv(self, point, leg):
    return self.chassisPose.toGlobal(self.legPose[leg].toGlobal(point))

  #Invokes the IK routines for a given leg on global coordinates
  def getAngles(self, point, leg):
    return self.legs[leg].IK(self.envToLeg(point, leg))
