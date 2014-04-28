import numpy as np
import math


def DHTransformMat(d, theta, r, alpha):
  sinT = np.sin(theta)
  cosT = np.cos(theta)
  sinA = np.sin(alpha)
  cosA = np.cos(alpha)
  return np.array([[cosT, -sinT*cosA, sinT*sinA, r*cosT],
                   [sinT, cosT*cosA, -cosT*sinA, r*sinT],
                   [0, sinA, cosA, d],
                   [0, 0, 0, 1]])

def finishDHMatrix(theta, unfinishedMatrix):
  sinT = np.sin(theta)
  cosT = np.cos(theta)
  unfinishedMatrix[0][0] = cosT
  unfinishedMatrix[0][1] *= sinT
  unfinishedMatrix[0][2] *= sinT
  unfinishedMatrix[0][3] *= cosT
  unfinishedMatrix[1][0] = sinT
  unfinishedMatrix[1][1] *= cosT
  unfinishedMatrix[1][2] *= cosT
  unfinishedMatrix[1][3] *= sinT
  return np.array([[cosT, -sinT*cosA, sinT*sinA, r*cosT],
                   [sinT, cosT*cosA, -cosT*sinA, r*sinT],
                   [0, sinA, cosA, d],
                   [0, 0, 0, 1]])


alpha = math.pi/2
r = 1
theta = -math.pi/2
d = -.5

DHMat = DHTransformMat(d, theta, r, alpha)
endEffector = [0,0,0,1]
print np.dot(DHMat,endEffector)
