import numpy as np
from math import sin,cos,acos,asin,atan,sqrt,pi,atan2

def getUnfinishedDHMat(a, r, d):
  ca = cos(a)
  sa = sin(a)
  return np.array([[1,-ca,sa,r],[1,ca,-sa,r],[0,sa,ca,d],[0,0,0,1]])

def finishDHMat(mat, theta):
  ct = cos(theta)
  st = sin(theta)
  return mat*np.array([[ct,st,st,ct],[st,ct,ct,st],[1,1,1,1],[1,1,1,1]])


class leg():
  #Z axis angles
  a = [None]*4

  #Radius
  r = [None]*4

  #Normal offset
  d = [None]*4

  def __init__(self, alpha, r, d):
    self.a = alpha
    self.r = r
    self.d = d
    self.coxaDHP = getUnfinishedDHMat(alpha[0],r[0],d[0])
    self.femurDHP = getUnfinishedDHMat(alpha[1],r[1],d[1])
    self.tibiaDHP = getUnfinishedDHMat(alpha[2],r[2],d[2])

  #partial DH mats:
  coxaDHP = None
  femurDHP = None
  tibiaDHP = None

  def FKEndPoint(self, t):
    return np.dot(np.dot(finishDHMat(self.coxaDHP,t[0]),finishDHMat(self.femurDHP,t[1]),finishDHMat(self.tibiaDHP,t[2])),np.array([0,0,0,1]))[:3]

  def FKAllPoints(self, t):
    femur = finishDHMat(self.coxaDHP,t[0])
    tibia = np.dot(femur,finishDHMat(self.femurDHP,t[1]))
    tarsus =  np.dot(tibia,finishDHMat(self.tibiaDHP,t[2]))
    return (np.array([0,0,0]),np.dot(femur,np.array([0,0,0,1]))[:3],np.dot(tibia,np.array([0,0,0,1]))[:3],np.dot(tarsus,np.array([0,0,0,1]))[:3])

  #Does IK assuming point is relative to leg
  def IK(self, point):
    #Ignore Z value and use only the axial offsets, with the X and Y coords to get the coxa angle
    coxaAngle = np.arctan2(point[1],point[0])+np.arcsin((self.d[1]+self.d[2])/np.sqrt(point[0]**2+point[1]**2))   

    #Calculate the inverse coxaDH, and apply to the point to hop into the femur coordinate system
    femurPoint = np.dot(np.linalg.inv(finishDHMat(self.coxaDHP, coxaAngle)), np.append(point,[1]))

    #Now we should only have to care about Y and X
    #Only two possibilities from here on
    footDist = femurPoint[0]**2+femurPoint[1]**2
    innerTibiaAngle = acos((self.r[1]**2+self.r[2]**2-footDist)/(2*self.r[1]*self.r[2]))
    femurAngle = asin(self.r[2]*sin(innerTibiaAngle)/sqrt(footDist))+atan2(femurPoint[1],femurPoint[0])
    return (coxaAngle,femurAngle,innerTibiaAngle-pi)

