import numpy as np
import time

class leg():

  #Structural parameter for this leg.
  #All this junk needs to be initialized in the constructor
  #Servo angles
  t = [0]*4

  #Cos/Sin for the servo angles
  ct = [None]*4
  st = [None]*4
  
  #Z axis angles
  a = [None]*4

  #Cos/Sin for the alpha angles
  ca = [None]*4
  sa = [None]*4

  #Radius
  r = [None]*4

  #Normal offet
  d = [None]*4

  def __init__(self, theta, alpha, r, d):
    self.t = theta
    self.a = alpha
    self.r = r
    self.d = d
    self.doThetaTrig()
    self.doAlphaTrig()

  def doThetaTrig(self):
   self.ct = [np.cos(i) for i in self.t]
   self.st = [np.sin(i) for i in self.t]

  def doAlphaTrig(self):
   self.ca = [np.cos(i) for i in self.a]
   self.sa = [np.sin(i) for i in self.a]
    
  #Matrix magic
  chassisDH = None
  coxaDH = None
  femurDH = None
  tibiaDH = None

  coxaMatrix = None
  femurMatrix = None
  tibiaMatrix = None
  tarsusMatrix = None

  def updateAllMatrices(self):
    ct = self.ct
    st =self.st
    a = self.a
    ca = self.ca
    sa = self.sa
    r = self.r
    d = self.d

    self.chassisDH = np.array([[ ct[0], -st[0], 0, ct[0]*r[0]],
    [ st[0],  ct[0], 0, r[0]*st[0]],
    [   0,    0, 1,     d[0]],
    [   0,    0, 0,      1]])
 
    self.coxaDH = np.array([[ ct[1], -ca[1]*st[1],  sa[1]*st[1], ct[1]*r[1]],
    [ st[1],  ca[1]*ct[1], -ct[1]*sa[1], r[1]*st[1]],
    [   0,      sa[1],      ca[1],     d[1]],
    [   0,        0,        0,      1]])
 
    self.femurDH = np.array([[ ct[2], -st[2], 0, ct[2]*r[2]],
    [ st[2],  ct[2], 0, r[2]*st[2]],
    [   0,    0, 1,     d[2]],
    [   0,    0, 0,      1]])
 
    self.tibiaDH = np.array([[ ct[3], -st[3], 0,  0],
    [ st[3],  ct[3], 0,  0],
    [   0,    0, 1, d[3]],
    [   0,    0, 0,  1]])

    self.coxaMatrix = self.chassisDH
    self.femurMatrix = np.dot(self.coxaMatrix, self.coxaDH)
    self.tibiaMatrix = np.dot(self.femurMatrix, self.femurDH)
    self.tarsusMatrix = np.dot(self.tibiaMatrix, self.tibiaDH)
    
  def updateFinalMatrix(self):

    ct = self.ct
    st =self.st
    a = self.a
    ca = self.ca
    sa = self.sa
    r = self.r
    d = self.d


    self.tarsusMatrix = np.array([[ - ct[3]*(st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) - ct[2]*(ct[0]*ct[1] - st[0]*st[1])) - st[3]*(ct[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) + st[2]*(ct[0]*ct[1] - st[0]*st[1])), st[3]*(st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) - ct[2]*(ct[0]*ct[1] - st[0]*st[1])) - ct[3]*(ct[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0]) + st[2]*(ct[0]*ct[1] - st[0]*st[1])), ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0], ct[0]*r[0] + d[2]*(ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0]) + d[3]*(ct[0]*sa[1]*st[1] + ct[1]*sa[1]*st[0]) + ct[2]*r[2]*(ct[0]*ct[1] - st[0]*st[1]) + ct[0]*ct[1]*r[1] - r[1]*st[0]*st[1] - r[2]*st[2]*(ca[1]*ct[0]*st[1] + ca[1]*ct[1]*st[0])],
    [ ct[3]*(st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) + ct[2]*(ct[0]*st[1] + ct[1]*st[0])) + st[3]*(ct[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) - st[2]*(ct[0]*st[1] + ct[1]*st[0])), ct[3]*(ct[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) - st[2]*(ct[0]*st[1] + ct[1]*st[0])) - st[3]*(st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1]) + ct[2]*(ct[0]*st[1] + ct[1]*st[0])), sa[1]*st[0]*st[1] - ct[0]*ct[1]*sa[1], r[0]*st[0] - d[2]*(ct[0]*ct[1]*sa[1] - sa[1]*st[0]*st[1]) - d[3]*(ct[0]*ct[1]*sa[1] - sa[1]*st[0]*st[1]) + ct[2]*r[2]*(ct[0]*st[1] + ct[1]*st[0]) + ct[0]*r[1]*st[1] + ct[1]*r[1]*st[0] + r[2]*st[2]*(ca[1]*ct[0]*ct[1] - ca[1]*st[0]*st[1])],
    [ ct[2]*sa[1]*st[3] + ct[3]*sa[1]*st[2], ct[2]*ct[3]*sa[1] - sa[1]*st[2]*st[3], ca[1], d[0] + d[1] + ca[1]*d[2] + ca[1]*d[3] + r[2]*sa[1]*st[2]],
    [ 0, 0, 0, 1]])

  def move(self, thetas):
    self.t = thetas
    self.doThetaTrig()
    self.updateFinalMatrix()
    return np.dot(self.tarsusMatrix,np.array([self.r[3],0,0, 1]))[:3]

  def getAllJoints(self, thetas):
    self.t = thetas
    self.doThetaTrig()
    self.updateAllMatrices()
    coxa = np.dot(self.coxaMatrix,np.array([0,0,0, 1]))[:3]
    femur = np.dot(self.femurMatrix,np.array([0,0,0, 1]))[:3]
    tibia = np.dot(self.tibiaMatrix,np.array([0,0,0, 1]))[:3]
    tarsus =  np.dot(self.tarsusMatrix,np.array([self.r[3],0,0, 1]))[:3]
    return (coxa, femur, tibia, tarsus)
