import numpy as np
from math import sin,cos

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
    return np.dot(finishDHMat(self.coxaDHP,t[0]),finishDHMat(self.femurDHP,t[1]),finishDHMat(self.tibiaDHP,t[2]),np.array([0,0,0,1]))[:3]

  def FKAllPoints(self, t):
    femur = finishDHMat(self.coxaDHP,t[0])
    tibia = np.dot(femur,finishDHMat(self.femurDHP,t[1]))
    tarsus =  np.dot(tibia,finishDHMat(self.tibiaDHP,t[2]))
    return (np.array([0,0,0]),np.dot(femur,np.array([0,0,0,1]))[:3],np.dot(tibia,np.array([0,0,0,1]))[:3],np.dot(tarsus,np.array([0,0,0,1]))[:3])

  #Does IK assuming point is relative to leg
  def IK(self, point):
    #Get in reference frame of coxa
    #Find combined axial offset of tibia and femur
    #Project point into axis of coxa
    #inverseChassisDH = finishDHmat(self, ch
    #inFrame = np.dot(self.inverseChassisDH, np.append(point, [1]))
    #Ignore Z value, and use only the axial offsets, with the X and Y coords
    #coxaAngle = np.arctan2(inFrame[1],inFrame[0])+np.arcsin((d[2]+d[3])/np.sqrt(inFrame[0]**2+inFrame[1]**2))   
    #Calculate a new inverseCoxaDH, and project point into it
    pass
    #Now we should only have to care about Z and X


