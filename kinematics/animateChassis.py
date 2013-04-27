import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin,sqrt
from random import random

from chassis import *
from chassisParams import chassisParams
from pose import pose
from tripod import *
from targetHolder import *


fig = plt.figure()
ax = p3.Axes3D(fig)

cp = chassisParams()
c = chassis(cp)
numLegs = len(c.legs)
groundZ = -1.5


velocity = (.5,.5)
speed = sqrt(sum(velocity))
if speed == 0:
  dir = (0,0)
else:
  dir = (velocity[0]/speed,velocity[1]/speed)
angularVelocity = 3
stepSize = .5
startPose = c.chassisPose
(startX,startY,startZ) = startPose.position
nextPose = pose((startX+dir[0]*stepSize,startY+dir[1]*stepSize,startZ),(startPose.yaw+angularVelocity,startPose.pitch,startPose.roll))

targets = targetHolder(np.array([getNextStep(i, c, nextPose, groundZ) for i in xrange(numLegs)]))

angles = [c.getAngles(targets.targets[i], i) for i in range(numLegs)]
segments = c.getChassisSegments(angles)
lines = [ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in segments]

updates = 20

def update_lines(num, lines):
  #If we have reached the end of this step
  progress = num/float(updates)
  if num%updates is 0:
    currentPose = c.chassisPose
    (currX,currY,currZ) = currentPose.position
    nextPose = pose((currX+dir[0]*stepSize,currY+dir[1]*stepSize,currZ),(currentPose.yaw+angularVelocity,currentPose.pitch,currentPose.roll))

    targets.targets = np.array([getNextStep(i, c, nextPose, groundZ) if i in [2*j for j in xrange(numLegs/2)] else targets.targets[i] for i in xrange(numLegs)])
  elif num%updates is updates/2:
    currentPose = c.chassisPose
    (currX,currY,currZ) = currentPose.position
    nextPose = pose((currX+dir[0]*stepSize,currY+dir[1]*stepSize,currZ),(currentPose.yaw+angularVelocity,currentPose.pitch,currentPose.roll))

    targets.targets = np.array([getNextStep(i, c, nextPose, groundZ) if i in [2*j+1 for j in xrange(numLegs/2)] else targets.targets[i] for i in xrange(numLegs)])

    

  #Update position
  x = startX + progress*velocity[0]
  y = startY + progress*velocity[1]
  z = startZ

  c.chassisPose = pose((x,y,z),(startPose.yaw+progress*angularVelocity,startPose.pitch,startPose.roll))

  #Get IK
  newThetas = [c.getAngles(targets.targets[i], i) for i in range(numLegs)]

  #Get FK for drawing
  newSegments = c.getChassisSegments(newThetas)
  for line,data in zip(lines,newSegments):
    line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
    line.set_3d_properties([data[0][2],data[1][2]])



ax.set_xlim3d([-3.0, 3.0])
ax.set_ylim3d([-3.0, 3.0])
ax.set_zlim3d([-3.0, 0.0])


line_ani = animation.FuncAnimation(fig, update_lines, 2000, fargs=(lines,), interval=10, blit=False)

plt.show()
