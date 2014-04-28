import numpy as np
from math import pi
from numpy import cos
from numpy import sin

t1, t2, t3 = (pi/2, pi/2, 0)
cosT1, cosT2, cosT3 = cos(t1), cos(t2), cos(t3)
sinT1, sinT2, sinT3 = sin(t1), sin(t2), sin(t3)

a1, a2, a3 = (pi/2, 0, 0)
cosA1 = cos(a1)
sinA1 = sin(a1)

r1, r2, r3 = (1, 1.5, 2)
d1, d2, d3 = (0, 0, 0)

transformMat = np.array([[cosT3*(cosT1*cosT2 - cosA1*sinT1*sinT2) - sinT3*(cosT1*sinT2 + cosA1*cosT2*sinT1), -cosT3*(cosT1*sinT2 + cosA1*cosT2*sinT1) - sinT3*(cosT1*cosT2 - cosA1*sinT1*sinT2), sinA1*sinT1, d3*sinA1*sinT1 + r1*cosT1 + d2*sinA1*sinT1 + r2*cosT1*cosT2 + r3*cosT3*(cosT1*cosT2 - cosA1*sinT1*sinT2) - r3*sinT3*(cosT1*sinT2 + cosA1*cosT2*sinT1) - r2*cosA1*sinT1*sinT2],
  [ cosT3*(cosT2*sinT1 + cosA1*cosT1*sinT2) - sinT3*(sinT1*sinT2 - cosA1*cosT1*cosT2), -cosT3*(sinT1*sinT2 - cosA1*cosT1*cosT2) - sinT3*(cosT2*sinT1 + cosA1*cosT1*sinT2), - sinA1*cosT1, r1*sinT1 - d3*sinA1*cosT1 - d2*sinA1*cosT1 + r2*cosT2*sinT1 - r3*sinT3*(sinT1*sinT2 - cosA1*cosT1*cosT2) + r3*cosT3*(cosT2*sinT1 + cosA1*cosT1*sinT2) + r2*cosA1*cosT1*sinT2],
  [sinT3*sinA1*cosT2 + sinA1*cosT3*sinT2, cosT3*sinA1*cosT2 - sinA1*sinT2*sinT3, cosA1, d1 + d2*cosA1 + d3*cosA1 + r3*sinT3*sinA1*cosT2 + r2*sinA1*sinT2 + r3*sinA1*cosT3*sinT2],
  [0,0,0,1]])

endEffector = np.array([0,0,0,1])
print np.dot(transformMat, endEffector)
