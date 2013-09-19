from chassis import *
import numpy as np
import random
import math

c = chassis()
maxErr = 0
for x in range(10):
  thetas = [[random.random()*math.pi/2-math.pi/4, random.random()*math.pi/2-math.pi/4,-random.random()*math.pi/2] for i in xrange(6)]
  c.theta = thetas
  feet = c.getFeet()
  angles = [c.getAngles(feet[i],i) for i in range(len(feet))]
  print np.array(c.theta) - np.array(angles)

