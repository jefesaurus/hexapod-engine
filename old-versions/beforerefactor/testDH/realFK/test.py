from leg import *
from math import pi


theta = [0, 0, 0, pi/2]
alpha = [0, pi/2, 0, 0]
radius = [1., .5, 1.5, 2]
displacement = [0,0,0,0]

l1 = leg(theta, alpha, radius, displacement)

print l1.move(theta)
print l1.getAllJoints(theta)
